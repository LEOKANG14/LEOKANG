#노드의 속성
class Node:
    total_DR = 1
    total_UAV = 1
    #현재 노드 번호와 각 대상 노드들로의 거리들
    def __init__(self, node_num, distances):
        self.visited = False
        self.node_num = node_num
        self.distances = distances
        self.clustered = False
        self.return_node = False
        self.connected_return_node = False
        self.num_of_DR = Node.total_DR
        self.num_of_UAV = Node.total_UAV
        
    #현재 노드에서 특정 대상 노드까지의 거리 반환// getter
    def get_distance(self, index):
        return self.distances[index]
        
    #노드 방문 상태 설정// setter
    def set_visited(self, visited):
        self.visited = visited
        
    #노드 방문 여부 확인// True, False 반환, Boolean에서나 사용
    def is_visited(self):
        return self.visited

    #SK_num 뱉어내게
    def get_num_of_SK(self, SK_num):
        if SK_num == "DR":
            num_of_SK = self.num_of_DR     #백슬래시 쓰면 띄어쓰기 가능
        elif SK_num == "UAV":
            num_of_SK = self.num_of_UAV
        return num_of_SK        
    
        
    # 현 노드 기준으로 가장 가까운 노드 순서대로 정렬함 (거리, 노드인덱스)
    def init_nearest(self):
        nearest = [(self.distances[idx], idx) for idx in range(len(self.distances))]
        nearest.sort()
        self.nearest = nearest


