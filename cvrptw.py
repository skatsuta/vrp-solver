#!/usr/bin/env python3
"""
Solve the Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""

import argparse
import json
import pygraphviz as pgv

from ortools.constraint_solver import pywrapcp, routing_enums_pb2


def load_data_model(path: str) -> dict:
    """
    Load the data for the problem from path.
    """
    with open(path) as file:
        data = json.load(file)

    data['num_locations'] = len(data['time_windows'])
    data['num_vehicles'] = len(data['vehicle_capacities'])

    return data


def create_weight_callback(data: dict):
    """
    Create a callback to return the weight between points.
    """

    def weight_callback(from_node, to_node):
        """
        Return the weight between the two points.
        """
        return data['weights'][from_node][to_node]

    return weight_callback


def create_demand_callback(data: dict):
    """
    Create a callback to get demands at each location.
    """

    def demand_callback(from_node, _):
        """
        Return the demand.
        """
        return data['demands'][from_node]

    return demand_callback


def add_capacity_constraints(routing: pywrapcp.RoutingModel, data: dict,
                             demand_callback):
    """
    Add capacity constraints.
    """
    routing.AddDimensionWithVehicleCapacity(
        evaluator=demand_callback,
        slack_max=0,  # null slack
        # vehicle maximum capacities
        vehicle_capacities=data['vehicle_capacities'],
        fix_start_cumul_to_zero=True,  # start cumul to zero
        name='Capacity',
    )


def create_time_callback(data: dict):
    """
    Create a callback to get total times between locations.
    """

    def service_time(node: int) -> int:
        """
        Get the service time to the specified location.
        """
        return data['service_times'][node]

    def travel_time(from_node: int, to_node: int) -> int:
        """
        Get the travel times between two locations.
        """
        return data['weights'][from_node][to_node]

    def time_callback(from_node: int, to_node: int):
        """
        Return the total time between the two nodes.
        """
        serv_time = service_time(from_node)
        trav_time = travel_time(from_node, to_node)
        return serv_time + trav_time

    return time_callback


def add_time_window_constraints(routing: pywrapcp.RoutingModel, data: dict,
                                time_callback):
    """
    Add time window constraints.
    """
    time = 'Time'
    horizon = 120
    routing.AddDimension(
        evaluator=time_callback,
        slack_max=horizon,  # allow waiting time
        capacity=horizon,  # maximum time per vehicle
        # Don't force start cumul to zero. This doesn't have any effect in this example,
        # since the depot has a start window of (0, 0).
        fix_start_cumul_to_zero=False,
        name=time,
    )
    time_dimension = routing.GetDimensionOrDie(time)
    for loc_node, (open_time, close_time) in enumerate(data['time_windows']):
        index = routing.NodeToIndex(loc_node)
        time_dimension.CumulVar(index).SetRange(open_time, close_time)


def node_properties(
        routing: pywrapcp.RoutingModel,
        assignment: pywrapcp.Assignment,
        capacity_dimension: pywrapcp.RoutingDimension,
        time_dimension: pywrapcp.RoutingDimension,
        index: int,
) -> tuple:
    """
    Get a node's properties on the index.
    """
    node_index = routing.IndexToNode(index)
    load = assignment.Value(capacity_dimension.CumulVar(index))
    time_var = time_dimension.CumulVar(index)
    time_min, time_max = assignment.Min(time_var), assignment.Max(time_var)
    return (node_index, load, time_min, time_max)


def print_solution(data: dict, routing: pywrapcp.RoutingModel,
                   assignment: pywrapcp.Assignment):
    """
    Print routes on console.
    """
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        node_props = []

        while not routing.IsEnd(index):
            props = node_properties(routing, assignment, capacity_dimension,
                                    time_dimension, index)
            node_props.append(props)
            index = assignment.Value(routing.NextVar(index))

        props = node_properties(routing, assignment, capacity_dimension,
                                time_dimension, index)
        node_props.append(props)
        route_time = assignment.Value(time_dimension.CumulVar(index))
        route = "\n  -> ".join(['[Node %2s: Load(%s) Time(%2s, %s)]' % prop \
                                for prop in node_props])
        plan_output = f'Route for vehicle {vehicle_id}:\n  {route}\n' + \
            f'Load of the route: {props[1]}\nTime of the route: {route_time} min\n'
        print(plan_output)

        total_time += route_time

    print(f'Total time of all routes: {total_time} min')


def draw_network_graph(data: dict):
    """
    Draw a network graph of the problem.
    """
    weights = data['weights']
    demands = data['demands']
    time_windows = data['time_windows']
    n_loc = data['num_locations']
    graph = pgv.AGraph(directed=False)

    def _node(index: int) -> str:
        if index == 0:
            return f'{index}\nDepot'
        return f'{index}\nDemand: {demands[index]}\nRange: {time_windows[index]}'

    for i in range(n_loc):
        for j in range(i + 1, n_loc):
            weight = weights[i][j]
            graph.add_edge(_node(i), _node(j), weight=weight, label=weight)

    filename = 'network.png'
    graph.draw(filename, prog='dot')

    print(f'The network graph has been saved to {filename}.')


def draw_route_graph(data: dict, routing: pywrapcp.RoutingModel,
                     assignment: pywrapcp.Assignment):
    """
    Draw a route graph based on the solution of the problem.
    """
    weights = data['weights']
    demands = data['demands']
    time_windows = data['time_windows']
    graph = pgv.AGraph(directed=True)

    def _node(index: int) -> str:
        if index == 0:
            return f'{index}\nDepot'
        return f'{index}\nDemand: {demands[index]}\nRange: {time_windows[index]}'

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_index = assignment.Value(routing.NextVar(index))
            next_node_index = routing.IndexToNode(next_index)
            weight = weights[node_index][next_node_index]
            graph.add_edge(
                _node(node_index),
                _node(next_node_index),
                weight=weight,
                label=weight,
            )
            index = next_index

    filename = 'route.png'
    graph.draw(filename, prog='sfdp')

    print(f'The route graph has been saved to {filename}.')


def parse_args() -> argparse.Namespace:
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('path', help='JSON file path of data')
    parser.add_argument(
        '-g',
        '--graph',
        help='export images of the network and the routes of vehicles',
        action='store_true',
    )
    parser.add_argument(
        '--gls',
        help='enable Guided Local Search',
        action='store_true',
    )
    return parser.parse_args()


def main():
    """
    Entry point of the program.
    """
    # Parse command line arguments
    args = parse_args()

    # Instantiate the data problem
    data = load_data_model(args.path)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(
        data['num_locations'],
        data['num_vehicles'],
        data['depot'],
    )

    # Define weight of each edge
    weight_callback = create_weight_callback(data)
    routing.SetArcCostEvaluatorOfAllVehicles(weight_callback)

    # Add capacity constraints
    demand_callback = create_demand_callback(data)
    add_capacity_constraints(routing, data, demand_callback)

    # Add time window constraints
    time_callback = create_time_callback(data)
    add_time_window_constraints(routing, data, time_callback)

    # Set first solution heuristic (cheapest addition)
    search_params = pywrapcp.RoutingModel.DefaultSearchParameters()
    # pylint: disable=no-member
    search_params.first_solution_strategy = \
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    if args.gls:
        # pylint: disable=no-member
        search_params.local_search_metaheuristic = \
                routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        search_params.time_limit_ms = 30000

    # Solve the problem
    assignment = routing.SolveWithParameters(search_params)
    if not assignment:
        print('No solution found.')
        return

    # Print the solution
    print_solution(data, routing, assignment)

    # Draw network and route graphs
    if args.graph:
        print()
        draw_network_graph(data)
        draw_route_graph(data, routing, assignment)


if __name__ == '__main__':
    main()
