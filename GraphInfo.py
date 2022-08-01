from Node import Node

#특정 시점의 그래프의 배치 정보를 출력
class GraphInfo:
    #노드셋 정보들을 받음
    def __init__(self, nodes):
        self.nodes = nodes

    
    #노드셋에서 특정 대상 노드를 반환// getter
    def get_node(self, index) -> Node:     #return type, hint 지정
        return self.nodes[index]
    
    #그래프 내 노드의 개수
    def get_node_cnt(self):
        return len(self.nodes)

    def set_num_of_SK(self, which_SK, target_node, num_of_SK):
        if which_SK =='DR':
            self.nodes[target_node].num_of_DR =num_of_SK
        elif which_SK =='UAV':
            self.nodes[target_node].num_of_UAV =num_of_SK

    
    #그래프 내 각 노드에서 num_of_SK 감소
    def decrease_num_of_SK(self, which_SK, target_nodes):
        for node_idx in target_nodes:
            if which_SK =='DR':
                self.nodes[node_idx].num_of_DR -= 1
            elif which_SK =='UAV':
                self.nodes[node_idx].num_of_UAV -= 1



    #두 노드 사이의 거리
    def get_distance(self, n1, n2):
        return self.get_node(n1).get_distance(n2) #get_distance를 같은 용어로 씀
    
    #그래프 내 모든 노드 방문상태 초기화
    def init_node_visit_status(self):
        for node in self.nodes:
            node.set_visited(False)
                        
    #특정 노드에 대해서 가장 가까운 노드 순서가 되도록 초기화
    def init_nearest_nodes(self):        
        for node in self.nodes:
            node.init_nearest()
        
#    def get_dist(self, start, end):
#        return self.nodes[start].distances[end]
