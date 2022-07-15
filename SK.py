# SK의 속성
class SK:
    def __init__(self):    
        self.loading_time = 0
        self.SK_disable_nodes = [ ]
        self.SK_num = ""

                
    #노드 방문 상태 설정// setter
    def set_disable_nodes(self, node):
        self.SK_disable_nodes.append(node)
    
    def get_disable_nodes(self):
        return self.SK_disable_nodes

    def clone_disable_nodes(self):
        return self.SK_disable_nodes[:]        