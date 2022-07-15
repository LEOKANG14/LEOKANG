class Path:
    def __init__(self):
        self.dpt_idx = 0
        self.arv_idx = 0
        self.nodes = []
        self.left_parcel = 0

    def set_path(self, dpt_idx, nodes, arv_idx):
        self.dpt_idx = dpt_idx
        self.arv_idx = arv_idx
        if isinstance(nodes, list):     #nodes가 list형이면 nodes를 복사
            self.nodes = nodes[:]
        else:
            self.nodes =[]     #list형이 아니면, nodes라는 리스트를 만들고 첨부
            self.nodes.append(nodes)

    def __repr__(self) -> str:          #representation, 오버로딩, 클래스 객체 주소만 나오는데, 빼서 볼 수 있게
        li = []
        li.append(self.dpt_idx)
        li.extend(self.nodes)
        li.append(self.arv_idx)

        return str(li)