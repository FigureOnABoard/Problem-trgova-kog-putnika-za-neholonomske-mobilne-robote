from sys import *
import matplotlib.pyplot as plt
from Path_Data_Holder import Path_Data_Holder

node_dist = 5
mapWidth = 1000
mapHeight = 1000

paths_safe_area = {}
paths_sensitive_area = {}

# Prikaže sve čvorove na prvim n i prvih m koordinata kao plave križiće, neovisno o tome jesu li posječeni ili ne.
def plotFirstNMVertices(n, m):
    x = []
    y = []
    for i in range(n + 1):
        for j in range(m + 1):
            x.append(i * node_dist)
            y.append(j * node_dist)
    plt.plot(x, y, 'bx')

def plotVerticesOnInterval(x_start, y_start, x_end, y_end):
    x = []
    y = []
    for i in range(x_start, x_end + 1):
        for j in range(y_start, y_end + 1):
            x.append(i * node_dist)
            y.append(j * node_dist)
    plt.plot(x, y, 'b.')

# Prikaže sve čvorove na grafu kao plave križiće, neovisno o tome jesu li posječeni ili ne.
def plotAllVertices():
    x = []
    y = []
    for i in range(mapWidth // node_dist):
        for j in range(mapHeight // node_dist):
            x.append(i * node_dist)
            y.append(j * node_dist)
    plt.plot(x, y, 'bx')

def plotLabel(v, label=False):
    if label == True:
        label = f"({v[0]},{v[1]})"
        x = v[0] * node_dist
        y = v[1] * node_dist
        plt.annotate(label, (x, y), textcoords="offset points", xytext=(0,10), ha='center')
    return

# Prima početni čvor v i put do csv datoteke s parovima (orijentacija, indeks putanje)
# Prikaže na grafu posjećene čvorove, ali ne i putanje.
def plotVisitedVertices(v, data, graph, label=False):
    x = []
    y = []

    for k in range(0, len(data), 2):
        x.append(v[0] * node_dist)
        y.append(v[1] * node_dist)
        plotLabel(v, label)

        fi = data[k]
        index = data[k+1]
        v[0] +=  graph[fi % 16][index][0][0]
        v[1] += graph[fi % 16][index][0][1]
    x.append(v[0] * node_dist)
    y.append(v[1] * node_dist)
    plotLabel(v, label)
    
    plt.plot(x, y, 'rx')
    return

# Prikaže čitav put na koordinatnom sustavu s koordinatama čelija i sve zauzete čelije na tom putu
def plotPath(v, data, graph, cost_map, path_color='orange'):
    x = []
    y = []
    for k in range(0, len(data), 2):
        x_center = v[0] * node_dist
        y_center = v[1] * node_dist
        fi = data[k]
        index = data[k+1]

        occupied_cells = cost_map[fi % 16][index]
        for cell in occupied_cells:
            x.append(x_center + cell[0])
            y.append(y_center + cell[1])
        
        v[0] += graph[fi % 16][index][0][0]
        v[1] += graph[fi % 16][index][0][1]

    #plt.plot(x, y, 'y.')
    plt.plot(x, y, color=path_color, marker='.', linestyle='none', label=2)
    return

def plotCentersOfTrajectories(v, data, graph, cost_map):
    x = []
    y = []
    for k in range(0, len(data), 2):
        x_center = v[0] * node_dist
        y_center = v[1] * node_dist
        fi = data[k]
        index = data[k+1]

        x_trajectory_center = 0
        y_trajectory_center = 0
        number_of_occupied_cells = 0
        occupied_cells = cost_map[fi % 16][index]
        for cell in occupied_cells:
            x_trajectory_center += x_center + cell[0]
            y_trajectory_center += y_center + cell[1]
            number_of_occupied_cells += 1
        x_trajectory_center = round(x_trajectory_center / number_of_occupied_cells)
        y_trajectory_center = round(y_trajectory_center / number_of_occupied_cells)
        x.append(x_trajectory_center)
        y.append(y_trajectory_center)
        
        v[0] += graph[fi % 16][index][0][0]
        v[1] += graph[fi % 16][index][0][1]

    #plt.plot(x, y, 'y.')
    plt.plot(x, y, color='blue', marker='.', linestyle='none', label=2)
    return

def plotPathsAsLines(v, path, graph, cost_map, line_color='orange'):
    x = []
    y = []
    for k in range(0, len(path), 2):
        # Trenutni pocetni cvor
        x_center = v[0] * node_dist
        y_center = v[1] * node_dist
        x.append(x_center)
        y.append(y_center)
        
        # Težište putanje od trenutnog početnog čvora do idućeg čvora na putanji
        x_trajectory_center = 0
        y_trajectory_center = 0
        number_of_occupied_cells = 0
        fi = path[k]
        index = path[k+1]
        occupied_cells = cost_map[fi % 16][index]
        for cell in occupied_cells:
            x_trajectory_center += x_center + cell[0]
            y_trajectory_center += y_center + cell[1]
            number_of_occupied_cells += 1
        x_trajectory_center = round(x_trajectory_center / number_of_occupied_cells)
        y_trajectory_center = round(y_trajectory_center / number_of_occupied_cells)
        x.append(x_trajectory_center)
        y.append(y_trajectory_center)

        # Idući čvor
        v[0] += graph[fi % 16][index][0][0]
        v[1] += graph[fi % 16][index][0][1]
    # Poslijednji čvor u putu
    x.append(v[0] * node_dist)
    y.append(v[1] * node_dist)

    plt.plot(x, y, color=line_color)

    return

# Prima dvije loiste koordinata čvorova i prikazuje te čvorove na karti, uključujući i njihove oznake
def plotSpecificVertices(x_v, y_v, label=True):
    x = []
    y = []
    for i in range(len(x_v)):
        x.append(x_v[i] * node_dist)
        y.append(y_v[i] * node_dist)
    plt.plot(x, y, 'ro', label=2)
    for i in range(len(x_v)):
        v = [x_v[i], y_v[i]]
        plotLabel(v, label)

def plotPathsAsCurves(v_start, path, graph, edges_curves, curve_color='grey'):
    v = v_start.copy()
    for k in range(0, len(path), 2):
        fi = path[k]
        index = path[k + 1]
        ec = edges_curves[fi][index]
        x = ec[0].copy()
        y = ec[1].copy()
        for i in range(len(x)):
            x[i] += v[0] * node_dist
            y[i] += v[1] * node_dist
        plt.plot(x, y, color=curve_color)
        # Idući čvor
        v[0] += graph[fi % 16][index][0][0]
        v[1] += graph[fi % 16][index][0][1]
    return

def plotAllLatticesOnInterval(x_start, y_start, x_end, y_end, edges_curves, color='grey'):
    for x_v in range(x_start, x_end):
        for y_v in range(y_start, y_end):
            v_start = [x_v, y_v]
            for fi in range(16):
                for index in range(len(edges_curves[fi])):
                    ec = edges_curves[fi][index]
                    x = ec[0].copy()
                    y = ec[1].copy()
                    for i in range(len(x)):
                        x[i] += v_start[0] * node_dist
                        y[i] += v_start[1] * node_dist
                    plt.plot(x, y, color=color)

def plotStraightLines(ordered_points, line_color="green"):
    x = []
    y = []
    for p in ordered_points:
        x.append(p[0] * node_dist)
        y.append(p[1] * node_dist)
    plt.plot(x, y, 'o-', color=line_color)

def readVars(path):
    varVars = []
    f = open(path)
    line = f.readline()
    while line:
        varVars.append(line.split(','))
        line = f.readline()
    return varVars

def readCost(path, j):
    pathToFile = path + "csv_cost_" + str(j) + ".csv"
    f = open(pathToFile)

    varCost = []

    line = f.readline()
    while (line):
        varCost.append(line.split(','))
        line = f.readline()
    
    return varCost

def readEdge(path, fi, index):
    pathToFileX = path + "/x/csv_edge_x_" + str(fi) + "_" + str(index) + ".csv"
    pathToFileY = path + "/y/csv_edge_y_" + str(fi) + "_" + str(index) + ".csv"
    f_x = open(pathToFileX)
    f_y = open(pathToFileY)
    line_x = f_x.readline()
    line_y = f_y.readline()
    f_x.close()
    f_y.close()
    x_edge = line_x.split(',')
    y_edge = line_y.split(',')
    for i in range(len(x_edge)):
        x_edge[i] = float(x_edge[i])
        y_edge[i] = float(y_edge[i])
    return x_edge, y_edge

# Vraća graph, cost_map i edges_curves
def readParams(path_to_vars="params/vars/csv_vars.csv", path_to_costs="params/cost/", path_to_edges="params/edges"):
    # indexi elemenata vektora "graph" predstavljaju početne orijentacije (0-15)
    # svaki element vektora "graph" sadrži informacije o svim 
    # susjednim čvorovima u koje se može doći s dotičnom početnom orijentacijom
    # informacije o susjednim čvorovima sadržane su u parovima oblika: 
    # (x-y koordinate susjednog čvora + konačna orijentacija, cijena pripadajućeg segmenta)
    graph = [[] for i in range(16)]
    # indexi elemenata vektora "cost_map" predstavljaju početne orijentacije (0-15)
    # svaki element vektora "cost_map" sadrži vektore parova s
    # x-y koordinatama svih čelija kroz koje vozilo prolazi prilikom 
    # izvođenja pojedinog segmenta s dotičnom početnom orijentacijom
    cost_map = [[] for i in range(16)]
    # indexi elemenata vektora "graph" predstavljaju početne orijentacije (0-15)
    # Elementi vektora edges_curves sadrže 100 točaka koje čine putanju puta.
    # Elementi iz "edge_curves" imaju oblik [[x0,x1,x2,...,x99], [y0,y1,y2,...,y99]]
    # TODO poboljšati opis
    edges_curves = [[] for i in range(16)]
    
    varVars = readVars(path_to_vars)
    # m - broj redaka
    # n - broj stupaca
    m = len(varVars)
    n = len(varVars[0])
    for i in range(m):
        tmp = []
        tmp.append(int(varVars[i][1]))
        tmp.append(int(varVars[i][2]))
        tmp.append(int(varVars[i][3]))
        sf = float(varVars[i][4])
        
        graph[int(varVars[i][0])].append((tmp, sf))

        varCost = readCost(path_to_costs, i + 1)

        # mc - broj redaka u varCost
        # nc - broj stupaca u varCost
        mc = len(varCost)
        nc = len(varCost[0])

        vptmp = []
        for j in range(nc):
            vptmp.append((int(varCost[0][j]), int(varCost[1][j])))
        
        cost_map[int(varVars[i][0])].append(vptmp)

    for fi in range(16):
        for index in range(len(graph[fi])):
            x_edge, y_edge = readEdge(path_to_edges, fi, index)
            edges_curves[fi].append([x_edge, y_edge])

    return graph, cost_map, edges_curves

# Funkcija čita putanje iz datoteke input_file_name i sprema ih u globalne variable paths_safe_area i paths_sensitive_area
def readAllPaths(input_file_name):
    input_file = open(input_file_name, 'r')

    for i in range(3):
        input_file.readline()
    
    line = input_file.readline().replace('\n', '')
    if (line != "safe_area"):
        print("GREŠKA u funkciji readAllPaths()! U datoteci " + input_file_name + " u 4. retku je očekivana vrijednost \"safe_area\", a nalazi se vrijednost \"" + line + "\".")
        input_file.close()
        exit()
    line = input_file.readline().replace('\n', '')
    # safe_area
    while (line != "sensitive_area" and line != ""):
        # redak s ključevima
        split_line = line.split(',')
        delta_x = int(split_line[0])
        delta_y = int(split_line[1])
        fi_start = int(split_line[2])
        fi_finish = int(split_line[3])

        # redak s veličinom niza koji sadrži podatke o putanji
        line = input_file.readline().replace('\n', '')
        size = int(line)
        
        # redak s putanjom
        new_path = []
        line = input_file.readline().replace('\n', '')
        split_line = line.split(',')
        k = 0
        while (k < len(split_line)):
            new_path.append(int(split_line[k]))
            new_path.append(int(split_line[k + 1]))
            k += 2
        
        paths_safe_area[delta_x, delta_y, fi_start, fi_finish] = new_path

        # idući redak
        line = input_file.readline().replace('\n', '')
    
    if (line == ""):
        print("GREŠKA u funkciji readAllPaths()! U datoteci " + input_file_name + " nije pronađena vrijednost \"sensitive_area\"")
        input_file.close()
        exit()
    line = input_file.readline().replace('\n', '')
    # sensitive_area
    while (line != ""):
        # redak s ključevima
        split_line = line.split(',')
        state_start_index = int(split_line[0])
        state_finish_index = int(split_line[1])

        # redak s veličinom niza koji sadrži podatke o putanji
        line = input_file.readline().replace('\n', '')
        size = int(line)

        # redak s putanjom
        new_path = []
        line = input_file.readline().replace('\n', '')
        split_line = line.split(',')
        k = 0
        while (k < len(split_line)):
            new_path.append(int(split_line[k]))
            new_path.append(int(split_line[k + 1]))
            k += 2

        paths_sensitive_area[state_start_index, state_finish_index] = new_path

        # idući redak
        line = input_file.readline().replace('\n', '')
    
    input_file.close()
    return

# Iz datoteke koja ima točke zapisane u vitičastim zagradama se čita i zapisuje točke u liste listi.
# Npr. ako datoteke ima zapis {{2,5}, {5,5}, {10,10}, {2,5}}, podaci se spremaju u Python listu [[2,5], [5,5], [10,10], [2,5]]
def readPointsFromCurlyBrackets(filename):
    f = open(filename, 'r')
    line = f.readline()
    if (line[0] != '{' or line[-1] != '}'):
        print("Greška u readPointsFromCurlyBrackets()! Datoteka ne započinje s \'{\' i(li) ne završava s \'}\'")
        exit(1)
    # Uklanjanje rubnih zagrada
    line = line[1:-1]
    # Splitanje tako da u points_list završe podaci oblika "{x,y}"
    points_list = line.split(", ")
    # Pretvorba "{x,y} u [x,y]"
    for i in range(len(points_list)):
        points_list[i] = points_list[i][1:-1]
        points_list[i] = points_list[i].split(',')
        points_list[i][0] = int(points_list[i][0])
        points_list[i][1] = int(points_list[i][1])
    # Vraćanje rezultata [[x1,y1], [x2,y2], ...]
    return points_list

# Prima string koji sadrži vektor vektora cijelih brojeva u C++ formatu
# Na osnovu vektora stvara i vraća Python listu lista cijelih brojeva.
def transform_cpp_vector_to_python_list(v):
    if (v[-1] == '\n'):
        v = v[:-1]
    l = []
    # Uklanjanje rubnih zagrada
    v = v[1:-1]
    # Dodavanje elemata u listu listi
    points_cpp = v.split(", ")
    for p_cpp in points_cpp:
        coordinates = p_cpp[1:-1].split(',')
        p_python = []
        for c in coordinates:
            p_python.append(int(c))
        l.append(p_python)
    return l

# Prima string s vitičastim zagradama i na osnovu njih napravi i vrati listu
def get_list_from_curly_brackets(s):
    if (s[-1] == '\n'):
        s = s[:-1]
    l = []
    # Uklanjanje rubnih zagrada
    s = s[1:-1]
    s = s.split(", ")
    for number in s:
        l.append(int(number))
    return l

# Čita soluciju i relevantne podatke - duljinu i vrijeme trajanja algoritma - i vraća ih
def read_solution(file_name):
    f = open(file_name, 'r')
    line = f.readline()
    solution = get_list_from_curly_brackets(line.split(": ")[1][:-1])
    line = f.readline()
    cost = float(line.split(": ")[1])
    line = f.readline()
    time = line.split(": ")[1]
    execution_time_str = ""
    for i in range(len(time)):
        if (time[i] == ' '):
            break
        execution_time_str += time[i]
    execution_time = float(execution_time_str)
    f.close()
    return solution, cost, execution_time

# Prima datoteku s izračunatim putevima i na osnovu nje definira zadatak
def read_problem(file_name):
    f = open(file_name)
    # Linija s opisom datoteke
    f.readline()
    # Linija s čvorovima
    line = f.readline()[:-1]
    nodes = transform_cpp_vector_to_python_list(line)
    states = []
    for n in nodes:
        for i in range(16):
            states.append([n[0],n[1],i])
    # Zatvaramo datoteku jer Paths_Data_Holder ima svoju vlastitu funkciju za čitanje
    f.close()
    pdh = Path_Data_Holder()
    pdh.readAllPaths(file_name)
    # Vraćamo nodes, states i pdh
    return nodes, states, pdh

def main():
    # Postavljanje veličine grafa
    fig = plt.figure(figsize=(8, 8))

    # Za prikaz TSP rješenja
    """
    solution_nodes = [(108,100), (102,100), (102,99), (102,98), (101,98), (100,98), (100,99), (101,99), (101,100), (100,100), (100,110), (101,110), (102,110), (102,111), (101,111), (100,111), (100,112), (101,112), (102,112), (107,117), (108,117), (108,118), (107,118), (107,119), (108,119), (109,119), (109,118), (109,117), (117,109), (117,108), (118,108), (118,109), (119,109), (119,108), (119,107), (118,107), (117,107), (118,100), (119,100), (120,100), (120,99), (120,98), (119,98), (119,99), (118,99), (118,98), (108,100)]
    plotStraightLines(solution_nodes)
    # Prikaz svih vrhova kao plavih križića
    plotVerticesOnInterval(100 - 8, 98 - 8, 120 + 8, 119 + 8)
    # Prikaz početne točke
    plt.plot([node_dist * 108], [node_dist * 100], 'go')
    plt.axis("equal")
    plt.show()
    exit(0)
    """

    # Za testiranje radius mulitplier-a: prikaži sve elementarne latice iz točke C = (50,50) = (500px,500px)
    """
    graph, cost_map, edges_curves = readParams(path_to_vars="params/vars/csv_vars.csv", path_to_costs="params/cost/", path_to_edges="params/edges")
    # plotAllLatticesOnInterval(50, 50, 51, 51, edges_curves)
    path_x = "params/edges/x_x2/"
    path_y = "params/edges/y_x2/"
    for i in range(0, 22):
        for j in range(0, 22):
            fig = plt.figure(figsize=(8, 8))
            fx = open(path_x + "csv_edge_x_" + str(i) + "_" + str(j) + "_x2.csv", 'r')
            fy = open(path_y + "csv_edge_y_" + str(i) + "_" + str(j) + "_x2.csv", 'r')
            lx = fx.readline()
            ly = fy.readline()
            if (lx == ''):
                continue
            x = lx.split(',')
            y = ly.split(',')
            for k in range(len(x)):
                x[k] = 500 + float(x[k])
                y[k] = 500 + float(y[k])
            plt.plot(x, y)
            plotVerticesOnInterval(47, 47, 53, 53)
            plt.axis("equal")
            plt.show()
    exit(0)

    # Testiranje rezultata OR-Tools_ETSP_solver.py
    nodes = readPointsFromCurlyBrackets("../ETSP_results/ETSP_result.txt")
    nodes.append(nodes[0])
    x_v = []
    y_v = []
    for n in nodes:
        x_v.append(n[0])
        y_v.append(n[1])
    plotStraightLines(nodes, 'black')
    plotVerticesOnInterval(min(x_v) - 8, min(y_v) - 8, max(x_v) + 8, max(y_v) + 8)
    plotSpecificVertices(x_v, y_v)
    # Prikaz ponečtne točke
    plt.plot([node_dist * nodes[0][0]], [node_dist * nodes[0][1]], 'go')
    plt.axis("equal")
    plt.show()
    exit(0)
    """

    """
    # Učitavanje imena ulazne datoteke.
    # Ulazna datoteka je datoteke koja sadrži puut kroz čvorove koji se želi vizualizirati.
    # Ulanzna datoteka je napisana u comma-separated values (csv) formatu tako da svaki redak predstavlja
    # par (orijentacija, indeks putanje). Prvi redak sadrži podatke o orijentaciji i putanji iz prvog čvora,
    # a zadnji redak podatke o  orijentaciji i putanji iz predzadnjeg čvora.
    input_filename = argv[1]
    input_flie = "results/" + input_filename

    # v - trenutni čvor. U početku početni čvor.
    v = [4, 4]

    #plotAllVertices()
    #plt.show()

    graph, cost_map = readParams()

    #plotAllVertices()
    plotFirstNMVertices(n=13, m=10)
    plotPath(v.copy(), input_flie, graph, cost_map)
    plotVisitedVertices(v.copy(), input_flie, graph)

    plt.show()
    """

    # Dohvati problem i rješenje
    nodes, states, pdh = read_problem("../results_paths/primjeri_za_usporedbe-NO_backwards/random_8_nodes_on_20x20-02.txt")
    solution, cost, execution_time = read_solution("../GTSP_results/MA without OH results/NO_backwards/random_8_nodes_on_20x20-02-with_MA_FINISHED.txt")
    """
    solution = [12, 182, 48, 64, 172, 83, 108, 130, 34, 21, 149, 118, 12]
    cost = 567.311
    execution_time = 2.77499
    """
    graph, cost_map, edges_curves = readParams()
    x_v = []
    y_v = []
    for n in nodes:
        x_v.append(n[0])
        y_v.append(n[1])

    # Usporedba plotPathsAsLines() i plotPathsAsCurves()
    """
    for s1 in range(0, len(states)):
        for s2 in range(s1 + 1, len(states)):
            if not (states[s1][0] == states[s2][0] and states[s1][1] == states[s2][1]):
                p = pdh.getPathForStates(s1, s2, states)
                v_start = [states[s1][0], states[s1][1]]
                plotPathsAsLines(v_start.copy(), p, graph, cost_map, 'orange')
                #plotPath(v_start.copy(), p, graph, cost_map, 'grey')
    for s1 in range(0, len(states)):
        for s2 in range(s1 + 1, len(states)):
            if not (states[s1][0] == states[s2][0] and states[s1][1] == states[s2][1]):
                p = pdh.getPathForStates(s1, s2, states)
                v_start = [states[s1][0], states[s1][1]]
                plotPathsAsCurves(v_start.copy(), p, graph, edges_curves, 'grey')
    plotVerticesOnInterval(0, 0, 50, 50)
    """

    # Prikaz svih latica
    #plotAllLatticesOnInterval(5, 5, 7, 7, edges_curves)
    """
    # Prikaz samo puteva između čvorova za GTSP
    for s1 in range(0, len(states)):
        for s2 in range(s1 + 1, len(states)):
            if not (states[s1][0] == states[s2][0] and states[s1][1] == states[s2][1]):
                p = pdh.getPathForStates(s1, s2, states)
                v_start = [states[s1][0], states[s1][1]]
                plotPathsAsCurves(v_start.copy(), p, graph, edges_curves, 'grey')
    """
    # Prikaz rješenja na grafu kao crne linije
    for i in range(len(solution) - 1):
        s1 = solution[i]
        s2 = solution[i + 1]
        p = pdh.getPathForStates(s1, s2, states)
        v_start = [states[s1][0], states[s1][1]]
        plotPathsAsCurves(v_start.copy(), p, graph, edges_curves, 'black')

    """
    plotVerticesOnInterval(0, 0, 50, 50)

    lat_x = [0,0.250378796601468,0.500757423562798,0.751135047405045,1.00150954527230,1.25187692418167,1.50223078430645,1.75256182545155,2.00285739593101,2.25310108311145,2.50327234493957,2.75334618182330,3.00329284828379,3.25307760383502,3.50266050258076,3.75199622104052,4.00103392372859,4.24971716601089,4.49798383375365,4.74576611925500,4.99299053291604,5.23957795006165,5.48544369226371,5.73049764245160,5.97464439301653,6.21778342603012,6.45980932460252,6.70061201430483,6.94007703347402,7.17808583110839,7.41451609094932,7.64924208023146,7.88213502147105,8.11306348555177,8.34189380426089,8.56849050032798,8.79271673292372,9.01443489471810,9.23350657840825,9.44979373526059,9.66315877078504,9.87346509929692,10.0805776457583,10.2843633365563,10.4846915872950,10.6814347852338,10.8744687640435,11.0636732825576,11.2489324244487,11.4301351122039,11.6071754810469,11.7799532943123,11.9483743273378,12.1123507297678,12.2718013647982,12.4266521240638,12.5768362426246,12.7222944641467,12.8629754173305,12.9988357339227,13.1298402504005,13.2559621597415,13.3771831350063,13.4934934249785,13.6048919223012,13.7113862047393,13.8129925503809,13.9097359277635,14.0016499620777,14.0887768787541,14.1711674258813,14.2488807770312,14.3219844161830,14.3905540065331,14.4546739433853,14.5144345715215,14.5699346716535,14.6212829491577,14.6685927035496,14.7119852589333,14.7515889126518,14.7875387460060,14.8199764366379,14.8490500744147,14.8749139825528,14.8977285456142,14.9176600458738,14.9348805209727,14.9495675756669,14.9619043171985,14.9720791935599,14.9802858990739,14.9867232478566,14.9915952339142,14.9951107649854,14.9974837433771,14.9989329951717,14.9996822428541,14.9999600876882,15.0000000000000]
    lat_y = [0,-3.99252895383519e-05,-0.000317780908080696,-0.00106703747326590,-0.00251629650721273,-0.00488928072272672,-0.00840481640326648,-0.0132768060321302,-0.0197141897026179,-0.0279208941951365,-0.0380957689401431,-0.0504325083981929,-0.0651195606808715,-0.0823400225096094,-0.102271520863825,-0.125086081905912,-0.150949987988589,-0.180023623750314,-0.212461312486924,-0.248411144152499,-0.288014796489669,-0.331407350919112,-0.378717104929781,-0.430065382805332,-0.485566346598225,-0.545326809320912,-0.609446052363377,-0.678015649168064,-0.751119266423159,-0.828832617688156,-0.911223165113158,-0.998350082269141,-1.09026411724133,-1.18700749545410,-1.28861384208822,-1.39510812566688,-1.50650662425808,-1.62281691559966,-1.74403789229882,-1.87015980309306,-2.00116432098502,-2.13702463888046,-2.27770559316906,-2.42316381549295,-2.57334791274929,-2.72819867517302,-2.88764931214553,-3.05162571458676,-3.22004675220506,-3.39282457010398,-3.56986494370184,-3.75106763640941,-3.93632678352231,-4.12553125942499,-4.31856524060575,-4.51530848341608,-4.71563673746162,-4.91942243078687,-5.12653497892705,-5.33684130817881,-5.55020634339185,-5.76649349874669,-5.98556513939498,-6.20728319404066,-6.43150945126813,-6.65810616669449,-6.88693650047387,-7.11786497615466,-7.35075792620162,-7.58550088023889,-7.82191418668791,-8.05992298770374,-8.29938800914393,-8.54019071160568,-8.78221661209383,-9.02535564655261,-9.26950239820976,-9.51455634922859,-9.76042209207113,-10.0070095097206,-10.2542339237887,-10.5020162096289,-10.7502828776607,-10.9989661201924,-11.2480038230925,-11.4973395417225,-11.7469224405853,-11.9967072077543,-12.2466538751658,-12.4967285146580,-12.7468997938573,-12.9971435055850,-13.2474382052408,-13.4977692569946,-13.7481231158565,-13.9984904933622,-14.2488649896732,-14.4992426117940,-14.7496212368558,-15.0000000000000]
    v_start = [5,45]
    for i in range(len(lat_x)):
        lat_x[i] += v_start[0] * node_dist
        lat_y[i] += v_start[1] * node_dist
    plt.plot(lat_x, lat_y, 'grey')

    for i in range(len(solution) - 1):
        s_start = solution[i]
        s_finish = solution[i+1]
        p = pdh.getPathForStates(s_start, s_finish, states)
        v_start = states[s_start][:2]
        plotPathsAsLines(v_start.copy(), p, graph, cost_map, 'black')
    """
    """
    for k in paths_safe_area.keys():
        p = paths_safe_area[k]
        plotPath(v_first.copy(), p, graph, cost_map)
    for k in paths_safe_area.keys():
        p = paths_safe_area[k]
        plotVisitedVertices(v_first.copy(), p, graph)
    
    
    k = paths_safe_area.keys()[0]
    print("k = ", k)
    p = paths_safe_area[k]
    print(p)
    plotVisitedVertices()
    """

    # Prikaz svih vrhova kao plavih kružića
    plotVerticesOnInterval(min(x_v) - 8, min(y_v) - 8, max(x_v) + 8, max(y_v) + 8)

    plotSpecificVertices(x_v, y_v)
    
    # Prikaz početne točke
    plt.plot([node_dist * nodes[0][0]], [node_dist * nodes[0][1]], 'go')
    
    plt.axis("equal")
    fig.suptitle("random_8_nodes_on_20x20-02", fontweight="bold")
    plt.title("b) MA result (cost: " + str(cost) + "; execution time: " + str(execution_time) + " s)\nNo backwards")
    plt.show()


if __name__ == "__main__":
    main()