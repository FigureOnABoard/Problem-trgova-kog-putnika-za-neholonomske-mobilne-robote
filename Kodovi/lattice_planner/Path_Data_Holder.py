

class Path_Data_Holder:
    def __init__(self):
        self.paths_safe_area = {}
        self.paths_sensitive_area = {}
        self.node_dist = 5
        self.r = 3 * self.node_dist

    # Pretpostavka je da virtualni prikaz mape ima dimenzije 1000x1000 i da se proteže po čelijama 0-999. (0 i 999 su granice/rubovi preko kojih se ne može)
    # Kako neki putevi koji su optimalni na području udaljenom od rubova mape nisu izvedivi na području blizu ruba mape, duljine se spremaju u costs
    # samo ako nisu previše blizu rubovima mape. Pretpostavljamo da se stanje nalazi blizu ruba ako je od nekog ruba udaljena za 2r ili manje, gdje je 2r
    # dvostruka duljina maksimalnog radijusa skretanja r koji definira fizička ograničenost robota.
    def check_if_state_near_edge(self, state):
        n = self.node_dist * state[0]
        m = self.node_dist * state[1]
        if (m <= 2 * self.r or n <= 2 * self.r or m >= 1000 - 2 * self.r or n >= 1000 - 2 * self.r):
            return True
        return False

    # Funkcija čita putanje iz datoteke input_file_name i sprema ih u globalne variable paths_safe_area i paths_sensitive_area
    def readAllPaths(self, input_file_name):
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
            
            self.paths_safe_area[delta_x, delta_y, fi_start, fi_finish] = new_path

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

            self.paths_sensitive_area[state_start_index, state_finish_index] = new_path

            # idući redak
            line = input_file.readline().replace('\n', '')
        
        input_file.close()
        return
    
    def getPathForStatesInSafeArea(self, state_start, state_finish):
        delta_x = state_finish[0] - state_start[0]
        delta_y = state_finish[1] - state_start[1]
        fi_start = state_start[2]
        fi_finish = state_finish[2]
        return self.paths_safe_area[delta_x, delta_y, fi_start, fi_finish]
    
    def getPathForStatesInSensitiveArea(self, state_start_infex, state_finish_index):
        return self.paths_sensitive_area[state_start_infex, state_finish_index]

    # Vraća podatke o putu između stanja state_start i state_finish
    def getPathForStates(self, state_start_index, state_finish_index, states):
        state_start = states[state_start_index]
        state_finish = states[state_finish_index]
        if (self.check_if_state_near_edge(state_start) == False and self.check_if_state_near_edge(state_finish) == False):
            return self.getPathForStatesInSafeArea(state_start, state_finish)
        else:
            return self.getPathForStatesInSensitiveArea(state_start_index, state_finish_index)
