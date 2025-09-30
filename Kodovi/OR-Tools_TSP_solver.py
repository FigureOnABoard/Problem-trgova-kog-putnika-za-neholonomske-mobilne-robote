import math
import time
import sys

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

"""
Program koristi OR-Tools za rješavanje problema trgovačkog putnika.
Dobiveno rješenje sprema se u "GTSP_results/TSP_result.txt" kako bi ga dalje mogaao koristiti GTSP_Optimize_headings.

Program se pokreće: python OR-Tools_TSP_solver.py input_file
gdje je "input_file" datoteka koju je napravio Costs_Data_Holder koja sadrži podatke o problemu."
"""

def create_data_model():
    data = {}
    # Točke za posjetiti
    data["locations"] = [(110,98), (103,109), (107,106), (102,90), (109,110), (104,95), (103,96), (97,98)]
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data

# Prima string koji sadrži vektor vektora cijelih brojeva u C++ formatu
# Na osnovu vektora stvara i vraća Python listu lista cijelih brojeva.
def transform_cpp_vector_to_python_list(v):
    l = []
    # Uklanjanje rubnih zagrada
    v = v[1:-1]
    # Dodavanje elemata u listu listi
    points_cpp = v.split(', ')
    for p_cpp in points_cpp:
        coordinates = p_cpp[1:-1].split(',')
        p_python = []
        for c in coordinates:
            p_python.append(int(c))
        l.append(p_python)
    return l

# Prima datoteku (put i naziv) koji sadrži podatke koje je spremio Cost_Data_Holder.
# Iz datoteke pročita sve čvorove.
# Vraća riječnik koji sadrži listu čvorova (dobivenu iz datoteke), broj vozila (uvijek postavljen na 1) i indeks početnog čvora (uvijek postavljen na 0).
def read_problem_from_file(filename):
    # Čitanje točaka iz datoteke sa specifičnim formatom
    f = open(filename, 'r')
    f.readline()
    line = f.readline()
    f.close()
    if (line[-1] == '\n'):
        line = line[:-1]
    nodes = transform_cpp_vector_to_python_list(line)

    # Spremanje rezuzltata u riječnik
    data = {}
    data["locations"] = nodes
    data["num_vehicles"] = 1
    data["depot"] = 0

    # Vraćanje rezultata
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euklidska udaljenost - Reuzltat kori+jenovanja se množi s 10000 kako bi se uzela u obzir decimale koje mogu dobiti.
                # Množenjem decimalnih vrijednosti istim brojem ne mijenja koji je od njih veći u odnosu na drugi, a za OR-Tools je potrebno koristiti cijele vrijednosti.
                # Zato koristimo prvih 4 decimala kako bi osigurali točnost rezultata.
                distances[from_counter][to_counter] = int(10000 *
                    math.hypot((from_node[0] - to_node[0]), (from_node[1] - to_node[1]))
                )
    return distances

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    index = routing.Start(0)
    plan_output = "Route:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    print(plan_output)
    plan_output += f"Objective: {route_distance}m\n"

def save_solution(locations, execution_time, filename, manager, routing, solution):
    f = open(filename, 'w')
    f.write('Best solution: {')
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        node = locations[node_index]
        f.write('{' + str(node[0]) + ',' + str(node[1]) + "}, ")
        index = solution.Value(routing.NextVar(index))
    node_index = manager.IndexToNode(index)
    node = locations[node_index]
    f.write('{' + str(node[0]) + ',' + str(node[1]) + "}}\n")
    f.write("execution_time: " + str(execution_time) + " s")

    f.close()
    print("Solution saved to " + filename)

def main():
    if (len(sys.argv) < 2):
        print("GRESKA TIJEKOM POKRETANJA PROGRAMA! Nije predan dovoljan broj argumenata.")
        print("program se pokreće: python OR-Tools.py input_file")
        print("gdje je \"input_file\" datoteka koju je napravio Costs_Data_Holder koja sadrži podatke o problemu.")
        sys.exit(1)
    data = read_problem_from_file(sys.argv[1])
    
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["locations"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data["locations"])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    start_time = time.time()
    solution = routing.SolveWithParameters(search_parameters)
    end_time = time.time()
    execution_time = end_time - start_time

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)

    # Save solution to a file
    save_solution(data["locations"], execution_time, "GTSP_results/TSP_result.txt", manager, routing, solution)

if __name__ == "__main__":
    main()
