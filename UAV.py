from SK import SK

# UAV의 속성 (SK를 상속함)
class UAV(SK):
    SK_capa = 1   #UAV 1회 가능 방문 수(single visit)
    c_loading_time = 30
    def __init__(self):
        super().__init__()    
        self.loading_time = 30
        #self.SK_disable_nodes = [ ]
        self.SK_num = "UAV"

    