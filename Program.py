import copy
from CSVReader import CSVReader
from DR import DR
from SK import SK
from UAV import UAV
from GraphInfo import GraphInfo
from Node import Node
from Path import Path

INFINITE = 999999

class Program:

    def __init__(self):
        reader = CSVReader()     #CSV 파일 읽어옴
        self.tau_graph = reader.load_csv('tau_v2.csv')
        self.taup_graph = reader.load_csv('taup_v2.csv')
        self.tau_graph.init_nearest_nodes()     #nearest 노드로 정렬 초기화
        self.taup_graph.init_nearest_nodes()

        self.DR_path_list : list[Path] = []     #리스트 타입 지정
        self.UAV_path_list : list[Path] = []


    def get_available_DR_path(self, target_node):     #DR_path_list 중에 target 노드와 가장 가까운 마지막 노드를 가진 available_DR_path를 출력
        available_DR_path = None 
        min_dist = INFINITE
        
        for DR_path in self.DR_path_list:
            if DR_path.left_parcel == 0:
                continue
            last_node = DR_path.nodes[-1]
            dist = self.taup_graph.get_distance(last_node, target_node)
            if dist < min_dist:
                min_dist = dist
                available_DR_path = DR_path
        
        return available_DR_path


    def get_SK_node_list(self):     #DR_path_list와 UAV_path_list의 nodes를 녹여합쳐서 SK_node_list를 출력
        sk_node_list = []
        
        for DR_path in self.DR_path_list:
            sk_node_list.extend(DR_path.nodes[:])
        for UAV_path in self.UAV_path_list:
            sk_node_list.extend(UAV_path.nodes[:])
        sk_node_list = list(set(sk_node_list))     #set으로 중복을 제거
        
        return sk_node_list


    def greedy_truck_node_list(self, current_SK_node_list):     #사이드 킥 노드를 제외한 트럭 노드 리스트를 확보
        greedy_truck_nodes_list = [node.node_num for node in self.tau_graph.nodes]    #[0, 1, 2, 3, 4, ...]
        
        for SK_node_num in current_SK_node_list:
            greedy_truck_nodes_list.remove(SK_node_num)
        
        return greedy_truck_nodes_list    #[ 0, 1, 2, 4, 5, 6, 7, ...]


    def greedy_tsp(self, greedy_truck_nodes_list):     # 트럭 노드 리스트로 TSP 실행
        depot_node = greedy_truck_nodes_list[0]     # depot 0번 노드
        greedy_path = [depot_node]     #경로에 대한 번호 리스트
        visit_node = {0:True}
        node_cnt = len(greedy_truck_nodes_list)
        current_node = depot_node
        
        for j in range(node_cnt-1):
            min_dist = INFINITE
            next_node = None
            for i in range(node_cnt):
                if greedy_truck_nodes_list[i] in visit_node.keys():
                    continue
                cur_distance = self.tau_graph.get_distance(current_node, greedy_truck_nodes_list[i])
                if cur_distance < min_dist:
                    min_dist = cur_distance
                    next_node = greedy_truck_nodes_list[i]            
            visit_node[next_node] = True
            greedy_path.append(next_node)
            current_node = next_node    #current_node를 출발점 기준으로 고정
        greedy_path.append(0)
        print("truck_path", greedy_path)
        
        return greedy_path                
    

    def get_truck_cum_dist(self, greedy_path):     #트럭 노드 TSP에 따른 cum_dist 나열
        truck_path = greedy_path
        total_dist = 0
        truck_cum_dist = [total_dist]
        
        for idx in range(len(truck_path)-1):
            node = self.tau_graph.get_node(truck_path[idx])     #노드 뽑아내기
            total_dist += node.get_distance(truck_path[idx+1])     #다음 노드까지 거리 적산
            truck_cum_dist.append(total_dist)     #적산된 거리 나열
        
        return truck_cum_dist


    def is_check_node_available_in_dpt(self, check_node):     #check_node가 dpt로 가능한지(True) 출력
        res = True     #result의 약자
        
        for path in self.DR_path_list:
            if path.dpt_idx == check_node:     #check_node가 어느 path의 dpt_node이면 False
                res = False
        for path in self.UAV_path_list:
            if path.dpt_idx == check_node:
                res = False
        
        return res


    def is_check_node_available_in_arv(self, attachable_path : Path, check_node):     #check_node가 arv로 가능한지(True) 출력
        res = True
        
        for path in self.DR_path_list:
            if path == attachable_path:     #어떠한 path가 available_nodes이면 통과 
                continue
            if path.arv_idx == check_node:     #check_node가 어느 path의 arv_node이면 False
                res = False
        for path in self.UAV_path_list:
            if path == attachable_path:
                continue
            if path.arv_idx == check_node:
                res = False
        
        return res
    
    
    def decision_best_path(self, check_node_list):     #input_list가 들어올 때 앞 노드부터 넣어가면서 
        
        for check_node in check_node_list:
            if self.is_check_node_available_in_dpt(check_node):
                self.choose_SK_usage(check_node)     #check_node를 Truck, DR, UAV중에 하나로 배정한다


    def assign_node(self, path : Path, path_info):     #_path_info에 있는 노드를 path에 배정한다
        nodes = path_info[1]     #_path_info의 shortest_path를 선택
        
        if path == None:     #path가 존재하지 않으면(첫번째 배정이면)
            path = Path()    #type 선언
            start = nodes[0]
            end = nodes[-1]
            path.set_path(nodes[0],nodes[1:-1],nodes[-1])     #nodes를 path에 입력
            if path_info[0] == 'UAV':     #UAV로 선언
                path.left_parcel = 0     #left_parcel을 0으로 만들어줌 (원래 0이었음, 따라서 변화가 없음)
                self.UAV_path_list.append(path)
            elif path_info[0] == 'DR':    #DR로 선언
                path.left_parcel = 2     #left_parcel을 2로 만들어줌 (원래 0이었음, 3에서 하나 빠짐)
                self.DR_path_list.append(path)

        else:     #path가 존재하면(DR의 뒤에 붙는 배정이면)
            if path.dpt_idx == nodes[-1]:     #dpt와 arv가 뒤집어지면 !!!중요!!!, 원래 path의 interval nodes에 +1 씩 해주고
                start = nodes[0]
                end = nodes[-1]
            else:
                start = path.arv_idx
                end = nodes[-1]
            path.set_path(nodes[0],nodes[1:-1],nodes[-1])     #nodes를 path에 입력
            if path_info[0] == 'UAV':     ###질문4### 이부분 이상, 아예 UAV에는 배정이 되지 않아야 함/ 그대로 0이 되므로 유지
                path.left_parcel = 0
            elif path_info[0] == 'DR':
                path.left_parcel -= 1     #하나씩 빼줌

        return path_info[0], start, end     #-1을 위한 start, end 받음


    def assign_DR_UAV(self, DR_path, DR_path_info_when_DR, UAV_path, UAV_path_info_when_UAV):     #_path 와 _path_info 정보들을 넣어서 고른다
        if DR_path_info_when_DR == None and UAV_path_info_when_UAV == None:     #둘다 배정되지 않으면 현재 sequence 취소
            which_SK = ''
            start, end = -1, -1
        elif DR_path_info_when_DR == None:     #DR에 배정되지 않으면, UAV로 배정
            which_SK, start, end = self.assign_node(UAV_path, UAV_path_info_when_UAV)
        elif UAV_path_info_when_UAV == None:     #UAV에 배정되지 않으면, DR로 배정
            which_SK, start, end = self.assign_node(DR_path, DR_path_info_when_DR)
        elif DR_path_info_when_DR[2] <= UAV_path_info_when_UAV[2]:     #DR의 Total_dist가 UAV의 Total_dist 보다 작거나 같으면, DR로 배정
            which_SK, start, end = self.assign_node(DR_path, DR_path_info_when_DR)
        else:
            which_SK, start, end = self.assign_node(UAV_path, UAV_path_info_when_UAV)     #DR의 Total_dist가 UAV의 Total_dist 보다 크면, UAV로 배정

        return  which_SK, start, end


    def get_nearest_UAV_path(self, target_node)->Path:     #가장 가까운 UAV_path를 찾는다
        nearest_UAV_path = None
        min_dist = INFINITE
        
        for UAV_path in self.UAV_path_list:
            last_node = UAV_path.nodes[-1]
            dist = self.taup_graph.get_distance(last_node, target_node)
            if dist < min_dist:
                min_dist = dist
                nearest_UAV_path = UAV_path
        
        return nearest_UAV_path


    def get_nearest_DR_path_with_UAVs(self, target_node, nearest_UAV_node)->Path:     #UAV노드에서 가장 가까운 DR_path 찾는다
        min_dist = INFINITE
        nearest_DR_path = None
        nearest_DR_node_idx = 0
        
        for DR_path in self.DR_path_list:
            if DR_path.left_parcel== 0:     #DR_path의 left_parcel이 0이면 패스
                continue
            for node in DR_path.nodes:     #DR_path의 각 노드에 대해서
                dist = self.taup_graph.get_distance(node, target_node) + self.taup_graph.get_distance(node, nearest_UAV_node)     #노드 거리를 이어준다
                if dist < min_dist:
                    dist = min_dist
                    nearest_DR_path = DR_path
                    nearest_DR_node_idx = node
        
        return nearest_DR_node_idx, nearest_DR_path
    
    def update_num_of_SK_at_nodes(self, greedy_path : list):
        num_of_DR = Node.total_DR
        for node in greedy_path:
          for path in self.DR_path_list:
            if greedy_path.count(path.arv_idx) == 0:
                return
            is_reversed = greedy_path.index(path.dpt_idx) > greedy_path.index(path.arv_idx)
            if node == path.dpt_idx:
                num_of_DR = num_of_DR + 1 if is_reversed else num_of_DR - 1
            elif node == path.arv_idx:
                num_of_DR = num_of_DR - 1 if is_reversed else num_of_DR + 1
            self.tau_graph.set_num_of_SK('DR', node, num_of_DR)
            assert 0 <= num_of_DR <= Node.total_DR, 'num_of_DR : ' + str(num_of_DR)

        num_of_UAV = Node.total_UAV
        for node in greedy_path:
          for path in self.UAV_path_list:
            if greedy_path.count(path.arv_idx) == 0:
                return
            is_reversed = greedy_path.index(path.dpt_idx) > greedy_path.index(path.arv_idx)
            if node == path.dpt_idx:
                num_of_UAV = num_of_UAV + 1 if is_reversed else num_of_UAV - 1
            elif node == path.arv_idx:
                num_of_UAV = num_of_UAV - 1 if is_reversed else num_of_UAV + 1
            self.tau_graph.set_num_of_SK('UAV', node, num_of_UAV)
            assert 0 <= num_of_UAV <= Node.total_UAV, 'num_of_UAV : ' + str(num_of_UAV)
    
    '''
    매개변수로 전달받은 check_node에 대해서 DR, UAV 혹은 truck 중 어떤 경로에 포함될 것인지를 판단한다.
    [Pre-condition]
    - DR, UAV가 각각 방문하기 전 확정된 노드들이 존재한다.
    [Input]
    - check_node: DR, UAV가 방문할 것인지 판단할 노드
    [Output]
    - check_node가 DR, UAV 중 어떤 경로에 포함되거나, 모두 포함되지 않을지의 여부
    '''
    def choose_SK_usage(self, check_node):     #check_node를 Truck, DR, UAV중에 하나로 배정한다
        SK_node_cand_list = self.get_SK_node_list()     #[3,8]
        SK_node_cand_list.append(check_node)     #[3,8,15]

        #A. check_node가 트럭에 있을 때 총 거리
        truck_node_list_when_truck = self.greedy_truck_node_list(self.get_SK_node_list())
        greedy_path_when_truck = self.greedy_tsp(truck_node_list_when_truck)

        #B. check_node가 DR에 있을 때 총 거리
        truck_node_list_when_DR = self.greedy_truck_node_list(SK_node_cand_list)
        greedy_path_when_DR = self.greedy_tsp(truck_node_list_when_DR)

        self.update_num_of_SK_at_nodes(greedy_path_when_DR)

        DR_path = self.get_available_DR_path(check_node)
        DR_path_info_when_DR = self.calculate_DR_path(check_node, greedy_path_when_DR, DR_path)

        #C. check_node가 UAV에 있을 때 총 거리
        truck_node_list_when_UAV = self.greedy_truck_node_list(SK_node_cand_list)
        greedy_path_when_UAV = self.greedy_tsp(truck_node_list_when_UAV)
        UAV_path = self.get_nearest_UAV_path(check_node)
        UAV_path_info_when_UAV = self.calculate_UAV_path(check_node, greedy_path_when_UAV)
        if UAV_path != None:     #UAV_path가 존재하면     ###질문###UAV_path를 여러개를 봐야 함 (현재는 하나의 path만 봄)
            nearest_DR_node_idx, nearest_DR_path = self.get_nearest_DR_path_with_UAVs(check_node, UAV_path.nodes[-1])     #UAV에서 가장 가까운 DR_path 찾음
            if nearest_DR_path != None:     #가장 가까운 DR_path가 존재하면
                new_UAV_path = Path()
                new_UAV_path.set_path(UAV_path.dpt_idx, UAV_path.nodes, UAV_path.arv_idx)     #new_UAV_path를 만들어서 덮어씌운다
                new_UAV_path.nodes.append(nearest_DR_node_idx)
                UAV_path_info_when_UAV2 = self.calculate_UAV_path(check_node, greedy_path_when_UAV, new_UAV_path)     #new_UAV_path가 들어갔을때 계산
                if (UAV_path_info_when_UAV2 != None):     #UAV_path_info_when_UAV2가 존재하면     
                    if UAV_path_info_when_UAV == None or (UAV_path_info_when_UAV2[2] < UAV_path_info_when_UAV[2]):     #기존 UAV_path가 없거나 new_UAV_path가 더 작으면
                        UAV_path_info_when_UAV = UAV_path_info_when_UAV2     #덮어씌운다
                        nearest_DR_path.left_parcel -= 1     #left_parcel을 -1 해준다
                else:
                    UAV_path = None     #UAV_path_info_when_UAV2가 존재하지 않으면 None
            else:
                UAV_path = None     #가장 가까운 DR_path가 존재하지 않으면 None    
        which_SK, start, end = self.assign_DR_UAV(DR_path, DR_path_info_when_DR, UAV_path, UAV_path_info_when_UAV)
        # if which_SK =='DR':     #DR 일 때, start, end, target_sk_nodes의 인덱스
        #     start = greedy_path_when_DR.index(start)
        #     end = greedy_path_when_DR.index(end)
        #     target_SK_nodes = greedy_path_when_DR[start:end]
        # elif which_SK =='UAV':     #UAV 일 때, start, end, target_sk_nodes의 인덱스
        #     start = greedy_path_when_UAV.index(start)
        #     end = greedy_path_when_UAV.index(end)
        #     target_SK_nodes = greedy_path_when_UAV[start:end]
        # if which_SK != '':     #which_SK가 존재하면???
        #     self.tau_graph.decrease_num_of_SK(which_SK, target_SK_nodes)     #graph_info에 직접 접근
        
        
        

    
    def calculate_DR_path(self, check_node, greedy_path, avail_path):     #DR으로 배정될 때 계산 + 기존 DR에 붙을 때도 고려
        DR_pathes = [ ]
        DR_input = DR()     #객체(클래스) 선언

        if self.is_check_node_available_in_arv(avail_path, check_node) == False:     #check_node가 avail_path에 삽입 불가하면 None을 출력
            DR_path_info = None
        else:
            DR_path_info = self.choose_dpt_arv(DR_input, check_node, greedy_path, avail_path)     #check_node를 배정한다
            if DR_path_info[2] == 0:     # 경로를 찾지 못했을 경우에는 None 한다, total_SK_dist를 따져서 배정되지 못했을 때
                DR_path_info = None
        
        return DR_path_info     #출력값: SK_num, shortest_path, total_SK_dist

  
    def calculate_UAV_path(self, check_node, greedy_path, nearest_path = None):    #UAV으로 배정될 때 계산 
        UAV_pathes = [ ]
        UAV_input = UAV()   #객체 선언
        
        if self.is_check_node_available_in_arv(nearest_path, check_node) == False:
            UAV_path_info = None
        else:
            UAV_path_info = self.choose_dpt_arv(UAV_input, check_node, greedy_path, nearest_path)
            if UAV_path_info[2] == 0:
                UAV_path_info = None

        return UAV_path_info     #출력값: SK_num, shortest_path, total_SK_dist


    def choose_dpt_arv(self, SK : SK, target_node, greedy_path, avail_path : Path = None):     #dpt_node와 arv_node를 정하는 모듈
        middle_nodes = []     #middle_nodes를 리스트화 함
        
        if avail_path == None:     #avail_path 없으면, 즉 기존 path 없이 새로 생성
            dpt_node = self.get_dpt_node(SK, target_node, greedy_path)
        else:     #기존 path가 존재하면
            dpt_node = avail_path.dpt_idx
            middle_nodes = avail_path.nodes[:]
        available_nodes = self.search_available_arv_nodes(target_node, dpt_node, SK, greedy_path)
        if len(available_nodes) == 0:
            return None, None, 0

        shortest_path, total_SK_dist = self.get_arv_node(target_node, dpt_node, SK, greedy_path, available_nodes, middle_nodes)
        SK_num = SK.SK_num
        
        return SK_num, shortest_path, total_SK_dist


    def get_dpt_node(self, SK : SK, target_node, greedy_path):     #현재 노드에서 가장 가까운 트럭 노드를 찾는다
        dpt_node = None
        min_dist = INFINITE

        for truck_node_number in greedy_path:
            if self.tau_graph.get_node(truck_node_number).get_num_of_SK(SK.SK_num) == 0:
                continue
            cur_distance = self.taup_graph.get_distance(target_node, truck_node_number)
            if cur_distance < min_dist:
                min_dist = cur_distance
                current_node = truck_node_number
        
        dpt_node = current_node
        search_nodes = greedy_path[greedy_path.index(dpt_node):]
        for node_idx in search_nodes:
            num_of_SK = self.tau_graph.get_node(node_idx).get_num_of_SK(SK.SK_num) - 1
            self.tau_graph.set_num_of_SK(SK.SK_num, node_idx, num_of_SK)
            if num_of_SK == 0:
                break
        
        return dpt_node


    def search_available_arv_nodes(self, target_node, dpt_node, SK : SK, greedy_path):     #available_arv_nodes를 찾는다
        '''
        greedy_truck_path 중 dpt_node를 포함하며, bike의 disabled_nodes를 포함하지 않을 수 있는 arv 후보 목록을 찾는다.
        '''
        available_nodes = [ ]

        dpt_index = greedy_path.index(dpt_node)     # 현재 입력된 dpt_node의 인덱스

        right_nodes = greedy_path[dpt_index:]     # 뒤로 쭉 다 
        if isinstance(right_nodes, list):
            for i in right_nodes:
                if self.tau_graph.get_node(i).get_num_of_SK(SK.SK_num) == 0 :
                    break
                if i != dpt_node:
                    available_nodes.append(i)

        left_nodes = greedy_path[:dpt_index]     # 앞으로 쭉 다 
        if isinstance(left_nodes, list):
            left_nodes.reverse()
            for i in left_nodes:
                if self.tau_graph.get_node(i).get_num_of_SK(SK.SK_num) == 0 :
                    break
                available_nodes.append(i)        

        return available_nodes
    

    def get_arv_node(self, target_node, dpt_node, SK, greedy_path, available_nodes, middle_nodes = []):     #arv_node를 취한다
        '''
        dpt_node -> target_node -> arv_node 가 트럭 노드의 dpt~arv의 경로보다 짧을 수 있는 arv_node를 찾는다.
        '''
        shortest_path_candi = []
        shortest_path_candi.extend(middle_nodes)
        shortest_path_candi.append(target_node)
        #cand_node_list = ["cand_node_list:"]
        #arv_node_list = ["arv_node_list:"]
        
        disabled = SK.clone_disable_nodes()
        total_SK_dist = 0
    
        min_dist = INFINITE
        for node in available_nodes:   #available_nodes 에서 가장 짧은 노드 선택
            if node in disabled:
                continue

            cur_distance = self.taup_graph.get_distance(target_node, node)

            if cur_distance < min_dist:
                min_dist = cur_distance
                cand_node = node    #선택하여 cand_node로 선언

        #cand_node_list.append(cand_node)

        ## 거리 따져서 가능여부 결정해야 함, 바이크 배송거리 + total_loading_time < 트럭 배송거리 이어야 함
        SK_dist = 0
        SK_path_trying = [dpt_node]
        SK_path_trying.extend(shortest_path_candi)
        SK_path_trying.append(cand_node)
        total_loading_time = 0
        for idx in range(len(SK_path_trying)-1):   #bike_a_path_trying의 거리 산정
            cur_node = self.taup_graph.get_node(SK_path_trying[idx])
            SK_dist += cur_node.get_distance(SK_path_trying[idx+1])
            total_loading_time += SK.loading_time
        
        #truck_dist 산정
        truck_dist = abs(
            self.get_truck_cum_dist(greedy_path)[greedy_path.index(cand_node)] - 
            self.get_truck_cum_dist(greedy_path)[greedy_path.index(dpt_node)]
            )

        disabled.append(cand_node)
        if SK_dist + total_loading_time <= truck_dist:
            SK.set_disable_nodes(cand_node)    ############## 바이크 배송이 선택되지 않으면 순환되어야 하는데 그렇지 않음
            total_SK_dist = SK_dist + total_loading_time
            for interval_node in greedy_path:
                if greedy_path.index(interval_node) > greedy_path.index(dpt_node) : 
                    if greedy_path.index(interval_node) < greedy_path.index(cand_node):
                        SK.set_disable_nodes(interval_node)
                        disabled.append(interval_node)
                    else:
                        continue
                else:
                    continue

                if greedy_path.index(interval_node) < greedy_path.index(dpt_node) : 
                    if greedy_path.index(interval_node) > greedy_path.index(cand_node):
                        SK.set_disable_nodes(interval_node)
                        disabled.append(interval_node)
                    else:
                        continue
                else:
                    continue

            if greedy_path.index(dpt_node) > greedy_path.index(cand_node):   #순서 맞추기
                temp = dpt_node
                dpt_node = cand_node
                shortest_path_candi.reverse()
                cand_node = temp

            
            
        
        shortest_path = [dpt_node]
        shortest_path.extend(shortest_path_candi)
        shortest_path.append(cand_node)

        return shortest_path, total_SK_dist #, cand_node_list, bike_dist_list_no, truck_dist_list_no, bike_dist_list_yes, truck_dist_list_yes, arv_node_list