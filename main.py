#!/usr/bin/env python3

"""
Solve the Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def create_data_model() -> dict:
    """
    Store the data for the problem.
    """
    # Weights on each edge
    _weights = [
        [0, 6, 9, 8, 6, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7],
        [6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14],
        [9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 13, 13, 9, 18, 9],
        [8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 13, 6, 16],
        [6, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14],
        [3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 6, 9, 7, 7, 6, 12, 8],
        [6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5],
        [2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10],
        [3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6],
        [2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5],
        [6, 13, 4, 14, 13, 6, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4],
        [6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10],
        [4, 5, 13, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8],
        [4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6],
        [5, 12, 9, 13, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2],
        [9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9],
        [7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0],
    ]

    # The number of people to ride
    demands = [
        0, # depot
        1, 1,
        1, 1,
        1, 1,
        1, 1,
        1, 1,
        1, 1,
        1, 1,
        1, 1,
    ]

    capacities = [15, 15, 15]

    time_windows = [
        (0, 0),
        (75, 95), (75, 95), # 1, 2
        (60, 80), (45, 65), # 3, 4
        (0, 20), (55, 75), # 5, 6
        (0, 20), (10, 30), # 7, 8
        (0, 20), (75, 95), # 9, 10
        (85, 105), (5, 25), # 11, 12
        (15, 35), (10, 30), # 13, 14
        (45, 65), (30, 50), # 15, 16
    ]

    data = {'weights': _weights,
            'num_locations': len(demands),
            'num_vehicles': len(capacities),
            'depot': 0,
            'demands': demands,
            'vehicle_capacities': capacities,
            'time_windows': time_windows,
            # Time in minutes to spend at each location for service (e.g. pickup)
            'service_time': 5}
    return data

def create_weight_callback(data):
    """
    Create a callback to return the weight between points.
    """
    def weight_callback(from_node, to_node):
        """
        Return the weight between the two points.
        """
        return data['weights'][from_node][to_node]

    return weight_callback

def create_demand_callback(data):
    """
    Create a callback to get demands at each location.
    """
    def demand_callback(from_node, _):
        """
        Return the demand.
        """
        return data['demands'][from_node]

    return demand_callback

def add_capacity_constraints(routing, data, demand_callback):
    """
    Add capacity constraints.
    """
    routing.AddDimensionWithVehicleCapacity(
        evaluator=demand_callback,
        slack_max=0, # null slack
        vehicle_capacities=data['vehicle_capacities'], # vehicle maximum capacities
        fix_start_cumul_to_zero=True, # start cumul to zero
        name='Capacity',
    )

def create_time_callback(data):
    """
    Create a callback to get total times between locations.
    """
    def service_time(_):
        """
        Get the service time to the specified location.
        """
        return data['service_time']

    def travel_time(from_node, to_node):
        """
        Get the travel times between two locations.
        """
        return data['weights'][from_node][to_node]

    def time_callback(from_node, to_node):
        """
        Return the total time between the two nodes.
        """
        serv_time = service_time(from_node)
        trav_time = travel_time(from_node, to_node)
        return serv_time + trav_time

    return time_callback

def add_time_window_constraints(routing, data, time_callback):
    """
    Add Global Span constraint for time windows.
    """
    time = 'Time'
    horizon = 120
    routing.AddDimension(
        evaluator=time_callback,
        slack_max=horizon, # allow waiting time
        capacity=horizon, # maximum time per vehicle
        # Don't force start cumul to zero. This doesn't have any effect in this example,
        # since the depot has a start window of (0, 0).
        fix_start_cumul_to_zero=False,
        name=time,
    )
    time_dimension = routing.GetDimensionOrDie(time)
    for loc_node, (start_time, end_time) in enumerate(data['time_windows']):
        index = routing.NodeToIndex(loc_node)
        time_dimension.CumulVar(index).SetRange(start_time, end_time)

def node_properties(routing, assignment, capacity_dimension, time_dimension, index) -> tuple:
    """
    Get a node's properties on the index.
    """
    node_index = routing.IndexToNode(index)
    load = assignment.Value(capacity_dimension.CumulVar(index))
    time_var = time_dimension.CumulVar(index)
    time_min, time_max = assignment.Min(time_var), assignment.Max(time_var)
    return (node_index, load, time_min, time_max)

def print_solution(data, routing, assignment):
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
            props = node_properties(routing, assignment, capacity_dimension, time_dimension, index)
            node_props.append(props)
            index = assignment.Value(routing.NextVar(index))

        props = node_properties(routing, assignment, capacity_dimension, time_dimension, index)
        node_props.append(props)
        route_time = assignment.Value(time_dimension.CumulVar(index))
        route = "\n  -> ".join(['[Node %2s: Load(%s) Time(%2s, %s)]' % prop for prop in node_props])
        plan_output = f'Route for vehicle {vehicle_id}:\n  {route}\n' + \
            f'Load of the route: {props[1]}\nTime of the route: {route_time} min\n'
        print(plan_output)

        total_time += route_time

    print(f'Total time of all routes: {total_time} min')

def main(enable_guided_local_search: bool = False):
    """
    Entry point of the program.
    """
    # Instantiate the data problem
    data = create_data_model()

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
    if enable_guided_local_search:
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

if __name__ == '__main__':
    main(enable_guided_local_search=False)
