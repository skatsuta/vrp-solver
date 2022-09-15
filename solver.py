"""
Solve the Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""

import argparse
from collections import abc
from pathlib import Path
from typing import Literal, TypeAlias

import graphviz as gv
import yaml
from ortools.constraint_solver import pywrapcp as cp
from ortools.constraint_solver.routing_enums_pb2 import (
    FirstSolutionStrategy,
    LocalSearchMetaheuristic,
)

TransitCallback: TypeAlias = abc.Callable[[int, int], int]
UnaryTransitCallback: TypeAlias = abc.Callable[[int], int]

# Ref. https://graphviz.org/docs/layouts/
LayoutEngine: TypeAlias = Literal[
    "dot", "neato", "fdp", "sfdp", "circo", "twopi", "osage"
]


def main() -> None:
    """
    Entry point of the program.
    """

    # Parse command line arguments
    args = _parse_args()

    # Instantiate the data problem
    data = _load_data_model(args.path)

    # Create the Routing Index Manager and Routing Model
    manager = cp.RoutingIndexManager(
        data["num_locations"], data["num_vehicles"], data["depot"]
    )
    routing = cp.RoutingModel(manager)

    # Define weights of edges
    weight_callback_index = routing.RegisterTransitCallback(
        create_weight_callback(manager, data)
    )
    routing.SetArcCostEvaluatorOfAllVehicles(weight_callback_index)

    # Add capacity constraints
    demand_callback = _create_demand_callback(manager, data)
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    _add_capacity_constraints(routing, manager, data, demand_callback_index)

    # Add time window constraints
    time_callback_index = routing.RegisterTransitCallback(
        _create_time_callback(manager, data)
    )
    _add_time_window_constraints(routing, manager, data, time_callback_index)

    # Set first solution heuristic (cheapest addition)
    search_params = cp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = FirstSolutionStrategy.PATH_CHEAPEST_ARC
    if args.gls:
        search_params.local_search_metaheuristic = (
            LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        # NOTE: Since Guided Local Search could take a very long time, we set a
        # reasonable time limit
        search_params.time_limit.seconds = 30
    if args.verbose:
        search_params.log_search = True

    # Solve the problem
    assignment = routing.SolveWithParameters(search_params)
    if not assignment:
        print("No solution found.")
        return

    # Print the solution
    _print_solution(data, routing, manager, assignment)

    # Export network and route graphs
    if args.export_network_graph:
        _draw_network_graph(args.export_network_graph, data)
    if args.export_route_graph:
        _draw_route_graph(args.export_route_graph, data, routing, manager, assignment)


def _parse_args() -> argparse.Namespace:
    """
    Parse command line arguments.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "path", help="JSON or YAML file that represents a vehicle routing problem"
    )
    parser.add_argument(
        "--gls",
        help=(
            "enable Guided Local Search (Note: This could take a long time, so it's"
            " a good idea to use --gls with -v to see the progress of a search)"
        ),
        action="store_true",
    )
    parser.add_argument(
        "-n",
        "--export-network-graph",
        metavar="NETWORK_FILE",
        help="file to save an image of the given network",
    )
    parser.add_argument(
        "-r",
        "--export-route-graph",
        metavar="ROUTE_FILE",
        help="file to save an image of the the routes of vehicles",
    )
    parser.add_argument(
        "-v", "--verbose", help="enable verbose output", action="store_true"
    )
    return parser.parse_args()


def _load_data_model(path: str) -> dict:
    """
    Load the data for the problem from path.
    """

    with open(path, encoding="utf-8") as file:
        data = yaml.safe_load(file)

    data["num_locations"] = len(data["time_windows"])
    data["num_vehicles"] = len(data["vehicle_capacities"])

    return data


def create_weight_callback(
    manager: cp.RoutingIndexManager, data: dict
) -> TransitCallback:
    """
    Create a callback to return the weight between points.
    """

    def weight_callback(from_index: int, to_index: int) -> int:
        """
        Return the weight between the two points.
        """

        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["weights"][from_node][to_node]

    return weight_callback


def _create_demand_callback(
    manager: cp.RoutingIndexManager, data: dict
) -> UnaryTransitCallback:
    """
    Create a callback to get demands at each location.
    """

    def demand_callback(from_index: int) -> int:
        """
        Return the demand.
        """

        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    return demand_callback


def _add_capacity_constraints(
    routing: cp.RoutingModel,
    manager: cp.RoutingIndexManager,
    data: dict,
    demand_callback_index: int,
) -> None:
    """
    Add capacity constraints.
    """

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        slack_max=0,  # null capacity slack
        vehicle_capacities=data["vehicle_capacities"],  # vehicle maximum capacities
        fix_start_cumul_to_zero=True,  # start cumul to zero
        name="Capacity",
    )


def _create_time_callback(
    manager: cp.RoutingIndexManager, data: dict
) -> TransitCallback:
    """
    Create a callback to get total times between locations.
    """

    def time_callback(from_index: int, to_index: int) -> int:
        """
        Return the total time between the two nodes.
        """

        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        # Get the service time to the specified location
        serv_time = data["service_times"][from_node]
        # Get the travel times between two locations
        trav_time = data["weights"][from_node][to_node]

        return serv_time + trav_time

    return time_callback


def _add_time_window_constraints(
    routing: cp.RoutingModel,
    manager: cp.RoutingIndexManager,
    data: dict,
    time_callback_index: int,
) -> None:
    """
    Add time window constraints.
    """

    horizon = data["max_time"]
    routing.AddDimension(
        time_callback_index,
        slack_max=horizon,  # allow waiting time
        capacity=horizon,  # maximum time per vehicle
        # Don't force start cumul to zero. This doesn't have any effect in this example,
        # since the depot has a start window of (0, 0).
        fix_start_cumul_to_zero=False,
        name="Time",
    )
    time_dimension = routing.GetDimensionOrDie("Time")
    for loc_idx, (open_time, close_time) in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(loc_idx)
        time_dimension.CumulVar(index).SetRange(open_time, close_time)


def _print_solution(
    data: dict,
    routing: cp.RoutingModel,
    manager: cp.RoutingIndexManager,
    assignment: cp.Assignment,
) -> None:
    """
    Print the solution.
    """

    capacity_dimension = routing.GetDimensionOrDie("Capacity")
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        node_props = []

        while not routing.IsEnd(index):
            props = _node_properties(
                manager, assignment, capacity_dimension, time_dimension, index
            )
            node_props.append(props)
            index = assignment.Value(routing.NextVar(index))

        props = _node_properties(
            manager, assignment, capacity_dimension, time_dimension, index
        )
        node_props.append(props)
        route_time = assignment.Value(time_dimension.CumulVar(index))
        route = "\n  -> ".join(
            ["[Node %2s: Load(%s) Time(%2s, %s)]" % prop for prop in node_props]
        )
        plan_output = (
            f"Route for vehicle {vehicle_id}:\n  {route}\n"
            f"Load of the route: {props[1]}\nTime of the route: {route_time} min\n"
        )
        print(plan_output)

        total_time += route_time

    print(f"Total time of all routes: {total_time} min")


def _node_properties(
    manager: cp.RoutingIndexManager,
    assignment: cp.Assignment,
    capacity_dimension: cp.RoutingDimension,
    time_dimension: cp.RoutingDimension,
    index: int,
) -> tuple:
    """
    Get a node's properties corresponding to the index.
    """

    node_index = manager.IndexToNode(index)
    load = assignment.Value(capacity_dimension.CumulVar(index))
    time_var = time_dimension.CumulVar(index)
    time_min, time_max = assignment.Min(time_var), assignment.Max(time_var)
    return (node_index, load, time_min, time_max)


def _draw_network_graph(filename: str, data: dict) -> None:
    """
    Draw a network graph of the problem.
    """

    weights = data["weights"]
    demands = data["demands"]
    time_windows = data["time_windows"]
    n_loc = data["num_locations"]
    graph = gv.Digraph(name="network")

    def _node_label(index: int) -> str:
        if index == 0:
            return f"{index}\nDepot"
        return f"{index}\nDemand: {demands[index]}\nRange: {time_windows[index]}"

    for i in range(n_loc):
        for j in range(i + 1, n_loc):
            # Add nodes and edge to the graph
            name_i, name_j = f"node{i}", f"node{j}"
            graph.node(name=name_i, label=_node_label(i))
            graph.node(name=name_j, label=_node_label(j))
            graph.edge(name_i, name_j, label=str(weights[i][j]))

    _render(graph, filename, engine="dot")

    print(f"The network graph has been saved to {filename}.")


def _draw_route_graph(
    filename: str,
    data: dict,
    routing: cp.RoutingModel,
    manager: cp.RoutingIndexManager,
    assignment: cp.Assignment,
) -> None:
    """
    Draw a route graph based on the solution of the problem.
    """

    weights = data["weights"]
    demands = data["demands"]
    time_windows = data["time_windows"]
    graph = gv.Digraph(name="route")

    def _node_label(index: int) -> str:
        if index == 0:
            return f"{index}\nDepot"
        return f"{index}\nDemand: {demands[index]}\nRange: {time_windows[index]}"

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            next_index = assignment.Value(routing.NextVar(index))
            next_node_index = manager.IndexToNode(next_index)
            weight = weights[node_index][next_node_index]

            # Add nodes and edge to the graph
            name_cur, name_next = f"node{node_index}", f"node{next_node_index}"
            graph.node(name=name_cur, label=_node_label(node_index))
            graph.node(name=name_next, label=_node_label(next_node_index))
            graph.edge(name_cur, name_next, label=str(weight))

            index = next_index

    _render(graph, filename, engine="sfdp")

    print(f"The route graph has been saved to {filename}.")


def _render(graph: gv.Digraph, filename: str, engine: LayoutEngine) -> None:
    """Render the graph and write the image to a file."""

    # Defaults to PNG if filename has no file extension
    ext = Path(filename).suffix or ".png"
    graph.render(cleanup=True, format=ext[1:], outfile=filename, engine=engine)


if __name__ == "__main__":
    main()
