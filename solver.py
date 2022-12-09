"""
Solve the Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import graphviz as gv
import yaml
from ortools.constraint_solver import pywrapcp as cp
from ortools.constraint_solver.routing_enums_pb2 import (
    FirstSolutionStrategy,
    LocalSearchMetaheuristic,
)
from ortools.constraint_solver.routing_parameters_pb2 import RoutingSearchParameters


def main() -> None:
    """Entry point of the program."""

    # Parse command line arguments
    args = _parse_args()

    # Load a problem
    data = _load_problem(args.path)

    # Create a Routing Index Manager and Routing Model
    manager = cp.RoutingIndexManager(data.num_locations, data.num_vehicles, data.depot)
    routing = cp.RoutingModel(manager)

    # Define weights of edges
    _set_edge_weights(routing, manager, data)

    # Add capacity constraints
    _add_capacity_constraints(routing, manager, data)

    # Add time window constraints
    _add_time_window_constraints(routing, manager, data)

    # Configure routing search parameters
    search_params: RoutingSearchParameters = cp.DefaultRoutingSearchParameters()
    # Use cheapest addition as the first solution heuristic
    search_params.first_solution_strategy = FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.time_limit.seconds = args.time_limit
    if args.gls:
        search_params.local_search_metaheuristic = (
            LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
    if args.verbose:
        search_params.log_search = True

    # Solve the problem
    assignment: cp.Assignment = routing.SolveWithParameters(search_params)
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
    """Parse command line arguments."""

    parser = argparse.ArgumentParser(
        # Print default values in help message
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "path", help="JSON or YAML file that represents a vehicle routing problem."
    )
    parser.add_argument(
        "-t",
        "--time-limit",
        help="Limit in seconds to the time spent in the search.",
        type=int,
        default=30,
    )
    parser.add_argument(
        "--gls",
        help=(
            "Enable Guided Local Search (Note: This could take a long time,"
            " so it's a good idea to use -t/--time-limit with --gls"
            " to set a reasonable time limit in a search)"
        ),
        action="store_true",
    )
    parser.add_argument(
        "-n",
        "--export-network-graph",
        metavar="NETWORK_FILE",
        help=(
            "File path to save the image of the network graph."
            " File format is automatically determined by the file extension."
            " See https://graphviz.org/docs/outputs/ for the supported file formats."
        ),
    )
    parser.add_argument(
        "-r",
        "--export-route-graph",
        metavar="ROUTE_FILE",
        help=(
            "File path to save the image of the graph of the vehicle routes."
            " File format is automatically determined by the file extension."
            " See https://graphviz.org/docs/outputs/ for the supported file formats."
        ),
    )
    parser.add_argument(
        "-v", "--verbose", help="Enable verbose output.", action="store_true"
    )
    return parser.parse_args()


@dataclass(frozen=True)
class Problem:
    weights: list[list[int]]
    service_times: list[int]
    demands: list[int]
    time_windows: list[list[int]]
    max_time: int
    vehicle_capacities: list[int]
    depot: int

    @property
    def num_locations(self) -> int:
        return len(self.time_windows)

    @property
    def num_vehicles(self) -> int:
        return len(self.vehicle_capacities)


def _load_problem(path: str) -> Problem:
    """Load the data for the problem from path."""

    with open(path, encoding="utf-8") as file:
        data = yaml.safe_load(file)

    return Problem(**data)


def _set_edge_weights(
    routing: cp.RoutingModel, manager: cp.RoutingIndexManager, data: Problem
) -> None:
    """Set weights of edges defined in the problem to the routing model."""

    def _weight_callback(from_index: int, to_index: int) -> int:
        """Return the weight between the two nodes."""

        from_node: int = manager.IndexToNode(from_index)
        to_node: int = manager.IndexToNode(to_index)
        return data.weights[from_node][to_node]

    weight_callback_index: int = routing.RegisterTransitCallback(_weight_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(weight_callback_index)


def _add_capacity_constraints(
    routing: cp.RoutingModel, manager: cp.RoutingIndexManager, data: Problem
) -> None:
    """Add capacity constraints defined in the problem to the routing model."""

    def _demand_callback(from_index: int) -> int:
        """Return the demand at the node."""

        from_node: int = manager.IndexToNode(from_index)
        return data.demands[from_node]

    demand_callback_index: int = routing.RegisterUnaryTransitCallback(_demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        slack_max=0,  # Null capacity slack
        vehicle_capacities=data.vehicle_capacities,  # Vehicle maximum capacities
        fix_start_cumul_to_zero=True,  # Start cumul to zero
        name="Capacity",
    )


def _add_time_window_constraints(
    routing: cp.RoutingModel, manager: cp.RoutingIndexManager, data: Problem
) -> None:
    """Add time window constraints defined in the problem to the routing model."""

    def _time_callback(from_index: int, to_index: int) -> int:
        """Return the total time between the two nodes."""

        from_node: int = manager.IndexToNode(from_index)
        to_node: int = manager.IndexToNode(to_index)

        # Get the service time to the specified node
        serv_time = data.service_times[from_node]
        # Get the travel times between two nodes
        trav_time = data.weights[from_node][to_node]

        return serv_time + trav_time

    time_callback_index: int = routing.RegisterTransitCallback(_time_callback)
    horizon = data.max_time
    routing.AddDimension(
        time_callback_index,
        slack_max=horizon,  # Allow waiting time
        capacity=horizon,  # Maximum time per vehicle
        # Don't force start cumul to zero. This doesn't have any effect in this example,
        # since the depot has a start window of (0, 0).
        fix_start_cumul_to_zero=False,
        name="Time",
    )
    time_dimension: cp.RoutingDimension = routing.GetDimensionOrDie("Time")
    for loc_idx, (open_time, close_time) in enumerate(data.time_windows):
        index: int = manager.NodeToIndex(loc_idx)
        time_dimension.CumulVar(index).SetRange(open_time, close_time)


def _print_solution(
    data: Problem,
    routing: cp.RoutingModel,
    manager: cp.RoutingIndexManager,
    assignment: cp.Assignment,
) -> None:
    """Print the solution."""

    capacity_dimension: cp.RoutingDimension = routing.GetDimensionOrDie("Capacity")
    time_dimension: cp.RoutingDimension = routing.GetDimensionOrDie("Time")
    total_time = 0

    for vehicle_id in range(data.num_vehicles):
        index: int = routing.Start(vehicle_id)
        nodes: list[Node] = []

        while not routing.IsEnd(index):
            node = _create_node(
                manager, assignment, capacity_dimension, time_dimension, index
            )
            nodes.append(node)

            next_var: cp.IntVar = routing.NextVar(index)
            index = assignment.Value(next_var)

        # Add the last node in the route
        node = _create_node(
            manager, assignment, capacity_dimension, time_dimension, index
        )
        nodes.append(node)

        time_var: cp.IntVar = time_dimension.CumulVar(index)
        route_time: int = assignment.Value(time_var)
        route = "\n  -> ".join(
            (
                f"[Node {node.index:2d}: Load({node.load:2d})"
                f" Time({node.min_time:2d}, {node.max_time:3d})]"
            )
            for node in nodes
        )
        print(
            f"Route for vehicle {vehicle_id}:\n     {route}\n"
            f"Load of the route: {node.load}\nTime of the route: {route_time} min\n"
        )

        total_time += route_time

    print(f"Total time of all routes: {total_time} min")


@dataclass(frozen=True)
class Node:
    index: int
    load: int
    min_time: int
    max_time: int


def _create_node(
    manager: cp.RoutingIndexManager,
    assignment: cp.Assignment,
    capacity_dimension: cp.RoutingDimension,
    time_dimension: cp.RoutingDimension,
    index: int,
) -> Node:
    """Create a Node corresponding to the index."""

    node_index: int = manager.IndexToNode(index)

    cap_var: cp.IntVar = capacity_dimension.CumulVar(index)
    load: int = assignment.Value(cap_var)

    time_var: cp.IntVar = time_dimension.CumulVar(index)
    min_time: int = assignment.Min(time_var)
    max_time: int = assignment.Max(time_var)

    return Node(node_index, load, min_time, max_time)


def _draw_network_graph(filename: str, data: Problem) -> None:
    """Draw a network graph of the problem."""

    def _node_label(index: int) -> str:
        if index == 0:
            return f"Node {index}\nDepot"
        return (
            f"Node {index}\nDemand: {data.demands[index]}\n"
            f"Window: {data.time_windows[index]}"
        )

    graph = gv.Digraph(name="network")

    n_loc = data.num_locations
    for i in range(n_loc):
        for j in range(i + 1, n_loc):
            # Add nodes and edge to the graph
            name_i, name_j = f"node{i}", f"node{j}"
            graph.node(name=name_i, label=_node_label(i))
            graph.node(name=name_j, label=_node_label(j))
            graph.edge(name_i, name_j, label=str(data.weights[i][j]))

    _render(graph, filename, engine="dot")

    print(f"The network graph has been saved to {filename}.")


def _draw_route_graph(
    filename: str,
    data: Problem,
    routing: cp.RoutingModel,
    manager: cp.RoutingIndexManager,
    assignment: cp.Assignment,
) -> None:
    """Draw a route graph based on the solution of the problem."""

    def _node_label(index: int) -> str:
        if index == 0:
            return f"Node {index}\nDepot"
        return (
            f"Node {index}\nDemand: {data.demands[index]}\n"
            f"Window: {data.time_windows[index]}"
        )

    graph = gv.Digraph(name="route")

    for vehicle_id in range(data.num_vehicles):
        index: int = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            node_index: int = manager.IndexToNode(index)
            next_var: cp.IntVar = routing.NextVar(index)
            next_index: int = assignment.Value(next_var)
            next_node_index: int = manager.IndexToNode(next_index)
            weight = data.weights[node_index][next_node_index]

            # Add nodes and edge to the graph
            name_cur, name_next = f"node{node_index}", f"node{next_node_index}"
            graph.node(name=name_cur, label=_node_label(node_index))
            graph.node(name=name_next, label=_node_label(next_node_index))
            graph.edge(name_cur, name_next, label=str(weight))

            index = next_index

    _render(graph, filename, engine="sfdp")

    print(f"The route graph has been saved to {filename}.")


def _render(
    graph: gv.Digraph,
    filename: str,
    # Ref. https://graphviz.org/docs/layouts/
    engine: Literal["dot", "neato", "fdp", "sfdp", "circo", "twopi", "osage"],
) -> None:
    """Render the graph and write the image to a file."""

    # Defaults to PNG if filename has no file extension
    ext = Path(filename).suffix or ".png"
    graph.render(cleanup=True, format=ext[1:], outfile=filename, engine=engine)


if __name__ == "__main__":
    main()
