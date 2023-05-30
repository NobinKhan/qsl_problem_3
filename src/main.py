from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    data = {}
    data['locations'] = [
        (23.8728568, 90.3984184),  # Uttara Branch
        (23.8513998, 90.3944536),  # City Bank Airport
        (23.8330429, 90.4092871),  # City Bank Nikunja
        (23.8679743, 90.3840879),  # City Bank Beside Uttara Diagnostic
        (23.8248293, 90.3551134),  # City Bank Mirpur 12
        (23.827149, 90.4106238),   # City Bank Le Meridien
        (23.8629078, 90.3816318),  # City Bank Shaheed Sarani
        (23.8673789, 90.429412),   # City Bank Narayanganj
        (23.8248938, 90.3549467),  # City Bank Pallabi
        (23.813316, 90.4147498)    # City Bank JFP
    ]
    data['num_locations'] = len(data['locations'])
    data['depot'] = 0  # Start from Uttara Branch
    return data

def compute_distance(lat1, lon1, lat2, lon2):
    # Simple Euclidean distance calculation
    return ((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2) ** 0.5

def create_distance_matrix(data):
    distances = {}
    for from_node in range(data['num_locations']):
        distances[from_node] = {}
        for to_node in range(data['num_locations']):
            if from_node == to_node:
                distances[from_node][to_node] = 0
            else:
                lat1, lon1 = data['locations'][from_node]
                lat2, lon2 = data['locations'][to_node]
                distances[from_node][to_node] = compute_distance(lat1, lon1, lat2, lon2)
    return distances

def solve_tsp(data):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(data['num_locations'], 1, data['depot'])
  
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distances'][from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
  
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
  
    # Set 10 seconds as the maximum time allowed for the search.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 10
  
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
  
    # Return the optimized route.
    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))
    return route

def main():
    # Create the data.
    data = create_data_model()
    data['distances'] = create_distance_matrix(data)
  
    # Solve the TSP.
    route = solve_tsp(data)
  
    # Print the optimized route.
    for node in route:
        print(f"City Bank Branch {node+1}: {data['locations'][node]}")

if __name__ == '__main__':
    main()
