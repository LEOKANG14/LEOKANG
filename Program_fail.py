from math import dist
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
        reader = CSVReader() #CSV 파일 읽어옴
        self.tau_graph = reader.load_csv('tau_v2.csv')
        self.taup_graph = reader.load_csv('taup_v2.csv')
        self.tau_graph.init_nearest_nodes()  #nearest 노드로 정렬 초기화
        self.taup_graph.init_nearest_nodes()
        
        #이전 시퀀스에서 이미 SK 노드로 설정된 노드들
        '''
        dr_test_path.set_path(9,3,18)
        dr_test_path.left_parcel = 2
        uav_test_path = Path()
        uav_test_path.set_path(17,8,5)
        uav_test_path.left_parcel = 0
        '''
        #self.DR_node_list = []
        #self.UAV_node_list = []
        self.DR_node_list : list[Path] = []  #리스트 타입 지정, 힌트를 얻기 위함 (강타입/약타입, 파이썬은 약타입)
        self.UAV_node_list : list[Path] = []



#        self.new_graph = reader.load_csv('tau_v2.csv')  # tau_graph가 list가 아니라서 deep copy가 안되므로

    def get_available_DR_path(self, target_node):
        available_DR_path = None
        min_dist = INFINITE
        for DR_path in self.DR_node_list:
            if DR_path.left_parcel ==0:
                continue          # 3번 방문 충족되면 예외 처리 먼저 함 (비정상적인 특이 상황을 먼저 걸러냄)
            last_node = DR_path.nodes[-1]
            current_dist = self.taup_graph.get_distance(target_node,last_node)
            if current_dist < min_dist:
                min_dist = current_dist
                available_DR_path = DR_path              # 그냥 copy이다. import copy에서 copy.deepcopy()로 딥카피 사용 가능(영원불멸)
            
        return available_DR_path



    def get_SK_node_list(self):
        
        SK_node_list = []
        
        for DR_path in self.DR_node_list: #zip은 두 list중에 짧은 list의 len에 맞춰서만 돔
            SK_node_list.extend(DR_path.nodes[:])

        for UAV_path in self.UAV_node_list: #zip은 두 list중에 짧은 list의 len에 맞춰서만 돔
            SK_node_list.extend(UAV_path.nodes[:])
        
        
        SK_node_list = list(set(SK_node_list))
        return SK_node_list

        '''
        for i in self.DR_node_list:
            i.remove(i[0])
            i.remove(i[-1])
            SK_node_list.extend(i)

        for i in self.UAV_node_list:
            i.remove(i[0])
            i.remove(i[-1])
            SK_node_list.extend(i)
        '''

    # 트럭 노드 리스트 확보 (사이드 킥 제외)
    def greedy_truck_node_list(self, current_SK_node_list):  # current_SK_node_list = [3, 8, 15]  

        greedy_truck_nodes_list = [node.node_num for node in self.tau_graph.nodes]  # [0, 1, 2, 3, 4, ...]

        for SK_node_num in current_SK_node_list:
            greedy_truck_nodes_list.remove(SK_node_num)

        return greedy_truck_nodes_list  #[ 0, 1, 2, 4, 5, 6, 7, ...]

    
    # 트럭 노드 리스트로 TSP 실행
    def greedy_tsp(self, greedy_truck_nodes_list):
        depot_node = greedy_truck_nodes_list[0]  # depot 0번 노드
        greedy_path = [depot_node]  #경로에 대한 번호 리스트
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
            current_node = next_node   # current_node를 출발점 기준으로 고정

        greedy_path.append(0)

        print('truck_path',greedy_path)
        
        return greedy_path                

    
    # 트럭 노드 TSP에 따른 Cum_dist 나열
    def get_truck_cum_dist(self, greedy_path):
        truck_path = greedy_path
        total_dist = 0
        truck_cum_dist = [total_dist]

        for idx in range(len(truck_path)-1):
            node = self.tau_graph.get_node(truck_path[idx])  #노드 뽑아내기
            total_dist += node.get_distance(truck_path[idx+1])  #다음 노드까지 거리 적산
            truck_cum_dist.append(total_dist)  #적산된 거리 나열

        return truck_cum_dist
    
    def is_check_node_available_in_dpt(self, check_node):   #check_node가 이전 시퀀스의 dpt_node인지?  
        res = True

        for path in self.DR_node_list:
            if path.dpt_idx == check_node:
                res = False
        for path in self.UAV_node_list:
            if path.dpt_idx == check_node:
                res = False
        return res

    def is_check_node_available_in_arv(self, attachable_path : Path, paths , check_node):   #check_node가 이전 시퀀스의 dpt_node인지?  
        res = True

        for path in paths:
            if path == attachable_path:
                continue
            if path.arv_idx == check_node:
                res = False            
            
        return res


    

    def decision_best_path(self, check_node_list):
        for check_node in check_node_list:
            if self.is_check_node_available_in_dpt(check_node):
                self.choose_SK_usage(check_node)



    def assign_node(self, path : Path, path_info):
        nodes = path_info[1]
        if path == None:
            path = Path()
            path.set_path(nodes[0],nodes[1:-1], nodes[-1])
            if path_info[0] =='UAV':
                path.left_parcel = 0
                self.UAV_node_list.append(path)
            elif path_info[0] =='DR':
                path.left_parcel = 2
                self.DR_node_list.append(path)
        else:
            path.set_path(nodes[0],nodes[1:-1], nodes[-1])

            if path_info[0] == 'UAV':
                pass
            elif path_info[0] =='DR':
                path.left_parcel -= 1




    def assign_DR_UAV(self, DR_path, DR_path_info_when_DR, UAV_path, UAV_path_info_when_UAV):
        if DR_path_info_when_DR == None and UAV_path_info_when_UAV == None:
            return
        elif DR_path_info_when_DR == None:
            self.assign_node(UAV_path, UAV_path_info_when_UAV)
        elif UAV_path_info_when_UAV == None:
            self.assign_node(DR_path, DR_path_info_when_DR)
        elif DR_path_info_when_DR[2] <= UAV_path_info_when_UAV[2]:
            self.assign_node(DR_path, DR_path_info_when_DR)
        else:
            self.assign_node(UAV_path, UAV_path_info_when_UAV)
            

#target node랑 가장 가까운 UAV node 1개
    def get_nearest_UAV_path(self, target_node)->Path:  #return type 지정
        nearest_UAV_path = None
        min_dist = INFINITE
        for UAV_path in self.UAV_node_list:
            last_node = UAV_path.nodes[-1]
            current_dist = self.taup_graph.get_distance(target_node,last_node)
            if current_dist < min_dist:
                min_dist = current_dist
                nearest_UAV_path = UAV_path     
        return nearest_UAV_path         


#UAV 노드 2개랑 가장 가까운 DR node 1개
    def get_nearest_DR_path_with_UAVs(self, target_node, nearest_UAV_node)-> Path:
        min_dist = INFINITE
        nearest_DR_path = None
        nearest_DR_node_idx = 0
        for DR_path in self.DR_node_list:
            for node in DR_path.nodes:
                dist = self.taup_graph.get_distance(node,target_node) + self.taup_graph.get_distance(node,nearest_UAV_node)
                if dist < min_dist:
                    dist = min_dist
                    nearest_DR_path = DR_path  
                    nearest_DR_node_idx = node


        return nearest_DR_node_idx, nearest_DR_path




            


    '''
    매개변수로 전달받은 check_node에 대해서 DR, UAV 혹은 truck 중 어떤 경로에 포함될 것인지를 판단한다.
    [Pre-condition]
    - DR, UAV가 각각 방문하기 전 확정된 노드들이 존재한다.
    [Input]
    - check_node: DR, UAV가 방문할 것인지 판단할 노드
    [Output]
    - check_node가 DR, UAV 중 어떤 경로에 포함되거나, 모두 포함되지 않을지의 여부
    '''
    def choose_SK_usage(self, check_node):
        
        SK_node_cand_list = self.get_SK_node_list()  #[3,8]
        SK_node_cand_list.append(check_node)  #[3,8,15]

        #A. check_node가 트럭에 있을 때 총 거리
        truck_node_list_when_truck = self.greedy_truck_node_list(self.get_SK_node_list())
        greedy_path_when_truck = self.greedy_tsp(truck_node_list_when_truck)
        #truck_node_total_dist = self.get_mixed_dist(greedy_path_when_truck, DR_pathes_when_truck)

        #B. check_node가 DR에 있을 때 총 거리
        truck_node_list_when_DR = self.greedy_truck_node_list(SK_node_cand_list)
        greedy_path_when_DR = self.greedy_tsp(truck_node_list_when_DR)
        DR_path = self.get_available_DR_path(check_node)
       
        #DR_node_total_dist = self.get_mixed_dist(greedy_path_when_DR, DR_pathes_when_DR)

        #C. check_node가 UAV에 있을 때 총 거리
        truck_node_list_when_UAV = self.greedy_truck_node_list(SK_node_cand_list)
        greedy_path_when_UAV = self.greedy_tsp(truck_node_list_when_UAV)
        UAV_path_info_when_UAV = self.calculate_UAV_path(check_node, greedy_path_when_UAV)
        UAV_path = self.get_nearest_UAV_path(check_node)
        if UAV_path != None:
            nearest_DR_node_idx, nearest_DR_path = self.get_nearest_DR_path_with_UAVs(check_node, UAV_path.nodes[-1])
            if nearest_DR_path != None:
                new_UAV_path = Path()
                new_UAV_path.set_path(UAV_path.dpt_idx, UAV_path.nodes, UAV_path.arv_idx)
                new_UAV_path.nodes.append(nearest_DR_node_idx)
                UAV_path_info_when_UAV2 = self.calculate_UAV_path(check_node, greedy_path_when_UAV, new_UAV_path)
                if UAV_path_info_when_UAV2 != None:
                    if UAV_path_info_when_UAV2 == None or (UAV_path_info_when_UAV2[2] < UAV_path_info_when_UAV[2]):  #or 조건은 첫번쨰가 true면 뒤에꺼는 안 본다
                        UAV_path_info_when_UAV = UAV_path_info_when_UAV2
                        nearest_DR_path.left_parcel -= 1
                
                else:
                    UAV_path = None
            else:
                UAV_path = None

        self.assign_DR_UAV(DR_path, UAV_path_info_when_UAV, UAV_path, UAV_path_info_when_UAV)


    #DR으로 배정될 때 계산 + 기존 DR에 복수개의 배정이 가능할 때도 고려   
    def calculate_DR_path(self, check_node, greedy_path, avail_path):
        DR_pathes = [ ]
        DR_input = DR()   #객체 선언
    
        if self.is_check_node_available_in_arv(avail_path, self.DR_node_list, check_node) == False:   #check_node가 arv_node와 겹치는 경우 처리 
            DR_path_info = None
        else:
            DR_path_info = self.choose_dpt_arv(DR_input, check_node, greedy_path, avail_path)

            # 경로를 찾지 못했을 경우에는 False 한다
            if DR_path_info[1] == 0:
                DR_path_info = None

        return DR_path_info

    #UAV으로 배정될 때 계산   
    def calculate_UAV_path(self, check_node, greedy_path, nearest_path =  None):
        UAV_pathes = [ ]
        UAV_input = UAV()   #객체 선언
     
        if self.is_check_node_available_in_arv(nearest_path, self.UAV_node_list, check_node) == False:   #check_node가 arv_node와 겹치는 경우 처리
            UAV_path_info = None
        else:
            UAV_path_info = self.choose_dpt_arv(UAV_input, check_node, greedy_path, nearest_path)
            # 경로를 찾지 못했을 경우에는 False 한다
            if UAV_path_info[1] == 0:
                UAV_path_info = None

        return UAV_path_info

    def choose_dpt_arv(self, SK : SK, target_node, greedy_path, avail_path : Path = None):
        '''
        A. greedy_path 중 SK가 target_node로 출발할 dpt_node를 결정한다.
            - dpt_node = get_dpt_node(SK, target_node, greedy_path) 
            : greedy_path 중 disabled 노드를 제외한 target_node에서 가장 가까운 노드를 찾는다.
        B. greedy_path 중 SK가 target_node에서 도착할 arv_node를 결정한다.
            - avaiable_nodes = self.search_available_arv_nodes(sk_num, dpt_node, target_node, greedy_path)
            - arv_node = self.get_arv_node(sk_num, dpt_node, target_node, available_nodes)
        
        C. DR로 dpt_node -> target_node -> arv_node 이동 시 전체 경로의 거리를 계산한다.
            
        return (sk_num, dpt_node, target_node, arv_node, bike_dist)
        '''
        middle_nodes = []
        if avail_path == None:    
            dpt_node = self.get_dpt_node(SK, target_node, greedy_path)
        else:
            dpt_node = avail_path.dpt_idx
            middle_nodes = avail_path.nodes[:]

        available_path = self.search_available_arv_nodes(target_node, dpt_node, SK, greedy_path)
        shortest_path, total_SK_dist = self.get_arv_node(target_node, dpt_node, SK, greedy_path, available_path, middle_nodes)
        
        SK_num = SK.SK_num
        
        return SK_num, shortest_path, total_SK_dist

    def get_dpt_node(self, SK, target_node, greedy_path):
        dpt_node = None
        min_dist = INFINITE

        for truck_node_number in greedy_path:
            if truck_node_number in SK.SK_disable_nodes:
                continue

            cur_distance = self.taup_graph.get_distance(target_node, truck_node_number)

            if cur_distance < min_dist:
                min_dist = cur_distance
                current_node = truck_node_number
        
        dpt_node = current_node
        SK.set_disable_nodes(dpt_node)     #disable nodes에 입력      
        
        return dpt_node

    def search_available_arv_nodes(self, target_node, dpt_node, SK, greedy_path):
        '''
        greedy_truck_path 중 dpt_node를 포함하며, bike의 disabled_nodes를 포함하지 않을 수 있는 arv 후보 목록을 찾는다.
        '''
        available_path = [ ]    # second_node로 가능한 노드 리스트
        disabled = SK.clone_disable_nodes()
    
        ###### pivot algorithm ######
        start_index = 0
        end_index = len(greedy_path) - 1

        first_index = greedy_path.index(dpt_node)  # dpt_node의 인덱스

        for i in disabled:
            tgt_index = greedy_path.index(i)

            if tgt_index < first_index and tgt_index > start_index:
                start_index = tgt_index

            if tgt_index > first_index and tgt_index < end_index:
                end_index = tgt_index

        #start_node = greedy_path[start_index]
        #end_node = greedy_path[end_index]

        for i in greedy_path:
            if greedy_path.index(i) < start_index:
                continue
            if greedy_path.index(i) > end_index:
                continue
            if i == dpt_node:
                continue
            available_path.append(i)
        
        print("disable_nodes:", disabled)
        print("available_arv_path:", available_path)
        
        return available_path
    
    def get_arv_node(self, target_node, dpt_node, SK, greedy_path, available_path, middle_nodes = []):
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
        for node in available_path:   #available_path 에서 가장 짧은 노드 선택
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
            SK.SK_disable_nodes.append(cand_node)    ############## 바이크 배송이 선택되지 않으면 순환되어야 하는데 그렇지 않음
            total_SK_dist = SK_dist + total_loading_time
            for interval_node in greedy_path:
                if greedy_path.index(interval_node) > greedy_path.index(dpt_node) : 
                    if greedy_path.index(interval_node) < greedy_path.index(cand_node):
                        SK.SK_disable_nodes.append(interval_node)
                        disabled.append(interval_node)
                    else:
                        continue
                else:
                    continue

                if greedy_path.index(interval_node) < greedy_path.index(dpt_node) : 
                    if greedy_path.index(interval_node) > greedy_path.index(cand_node):
                        SK.SK_disable_nodes.append(interval_node)
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