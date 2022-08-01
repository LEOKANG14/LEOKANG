import csv
from turtle import distance
from Node import Node
from GraphInfo import GraphInfo

#Reader로 .csv파일을 입력되는 노드 그래프로 변환함
class CSVReader:
    def __init__(self):
        pass  #input 없는 클래스는 그냥 이렇게 쓴다
    
    def load_csv(self, path):
        nodes = [ ] #노드 리스트 생성

        with open(path, newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',')
            node_idx = 0
            for row in spamreader:
                distances = [int(x) for x in row]
                if len(distances) == 0:
                    break
                distances.append(distances[0])
                n = Node(node_idx, distances) #n을 노드로서 속성 부여
                node_idx += 1
                nodes.append(n) #노드 리스트에 노드 번호와 그 노드에서 각 대상 노드들로의 거리들을 넣음
            
        n = Node(node_idx, nodes[0].distances)
        nodes.append(n)


        graph_info = GraphInfo(nodes) #그래프 클래스에 완성된 노드 리스트를 넣음
        return graph_info