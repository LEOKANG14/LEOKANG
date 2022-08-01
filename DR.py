from SK import SK

# DR의 속성
class DR(SK):
    SK_capa = 3
    c_loading_time = 20
    def __init__(self):    
        super().__init__()    
        self.loading_time = 20
        #self.SK_disable_nodes = [ ]
        self.SK_num = "DR"
            