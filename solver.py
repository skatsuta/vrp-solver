"""
Solve the Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""

import argparse
from typing import Callable

import pygraphviz as pgv
import yaml
from ortools.constraint_solver.pywrapcp import (
    Assignment,
    DefaultRoutingSearchParameters,
    RoutingDimension,
    RoutingIndexManager,
    RoutingModel,
)
from ortools.constraint_solver.routing_enums_pb2 import (
    FirstSolutionStrategy,
    LocalSearchMetaheuristic,
)

TransitCallback = Callable[[int, int], int]
UnaryTransitCallback = Callable[[int], int]


def main() -> None:
    """
    Entry point of the program.
    """

    # Parse command line arguments
    args = parse_args()

    # Instantiate the data problem
    data = load_data_model(args.path)

    # Create the Routing Index Manager and Routing Model
    manager = RoutingIndexManager(
        data["num_locations"], data["num_vehicles"], data["depot"]
    )
    routing = RoutingModel(manager)

    # Define weight of each edge
    weight_callback_index = routing.RegisterTransitCallback(
        create_weight_callback(manager, data)
    )
    routing.SetArcCostEvaluatorOfAllVehicles(weight_callback_index)

    # Add capacity constraints
    demand_callback = create_demand_callback(manager, data)
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    add_capacity_constraints(routing, manager, data, demand_callback_index)

    # Add time window constraints
    time_callback_index = routing.RegisterTransitCallback(
        create_time_callback(manager, data)
    )
    add_time_window_constraints(routing, manager, data, time_callback_index)

    # Set first solution heuristic (cheapest addition)
    search_params = DefaultRoutingSearchParameters()
    # pylint: disable=no-member
    search_params.first_solution_strategy = FirstSolutionStrategy.PATH_CHEAPEST_ARC
    if args.gls:
        # pylint: disable=no-member
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
    print_solution(data, routing, manager, assignment)

    # Draw network and route graphs
    if args.graph:
        print()
        draw_network_graph(data)
        draw_route_graph(data, routing, manager, assignment)


def parse_args() -> argparse.Namespace:
    """
    Parse command line arguments.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="JSON file path of data")
    parser.add_argument(
        "-g",
        "--graph",
        help="export images of the network and the routes of vehicles",
        action="store_true",
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
        "-v", "--verbose", help="enable verbose output", action="store_true"
    )
    return parser.parse_args()


def load_data_model(path: str) -> dict:
    """
    Load the data for the problem from path.
    """

    with open(path) as file:
        data = yaml.safe_load(file)

    data["num_locations"] = len(data["time_windows"])
    data["num_vehicles"] = len(data["vehicle_capacities"])

    return data


def create_weight_callback(manager: RoutingIndexManager, data: dict) -> TransitCallback:
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


def create_demand_callback(
    manager: RoutingIndexManager, data: dict
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


def add_capacity_constraints(
    routing: RoutingModel,
    manager: RoutingIndexManager,
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


def create_time_callback(manager: RoutingIndexManager, data: dict) -> TransitCallback:
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


def add_time_window_constraints(
    routing: RoutingModel,
    manager: RoutingIndexManager,
    data: dict,
    time_callback_index: int,
) -> None:
    """
    Add time window constraints.
    """

    horizon = 120
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


def print_solution(
    data: dict,
    routing: RoutingModel,
    manager: RoutingIndexManager,
    assignment: Assignment,
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
            props = node_properties(
                manager, assignment, capacity_dimension, time_dimension, index
            )
            node_props.append(props)
            index = assignment.Value(routing.NextVar(index))

        props = node_properties(
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


def node_properties(
    manager: RoutingIndexManager,
    assignment: Assignment,
    capacity_dimension: RoutingDimension,
    time_dimension: RoutingDimension,
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


def draw_network_graph(
    data: dict, filename: str = "network.png", prog: str = "dot"
) -> None:
    """
    Draw a network graph of the problem.
    """

    weights = data["weights"]
    demands = data["demands"]
    time_windows = data["time_windows"]
    n_loc = data["num_locations"]
    graph = pgv.AGraph(directed=False)

    def _node(index: int) -> str:
        if index == 0:
            return f"{index}\nDepot"
        return f"{index}\nDemand: {demands[index]}\nRange: {time_windows[index]}"

    for i in range(n_loc):
        for j in range(i + 1, n_loc):
            weight = weights[i][j]
            graph.add_edge(_node(i), _node(j), weight=weight, label=weight)

    graph.draw(filename, prog=prog)

    print(f"The network graph has been saved to {filename}.")


def draw_route_graph(
    data: dict,
    routing: RoutingModel,
    manager: RoutingIndexManager,
    assignment: Assignment,
    filename: str = "route.png",
    prog="sfdp",
) -> None:
    """
    Draw a route graph based on the solution of the problem.
    """

    weights = data["weights"]
    demands = data["demands"]
    time_windows = data["time_windows"]
    graph = pgv.AGraph(directed=True)

    def _node(index: int) -> str:
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
            graph.add_edge(
                _node(node_index), _node(next_node_index), weight=weight, label=weight
            )
            index = next_index

    graph.draw(filename, prog=prog)

    print(f"The route graph has been saved to {filename}.")


if __name__ == "__main__":
    main()
