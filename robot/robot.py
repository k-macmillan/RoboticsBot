from node_manager import Node, NodeManager

class Robot():
    def __init__(self):
        self.nm = NodeManager()
        self.initNodes()


    def initNodes(self):
        self.nm.add_node(Wheels())
        self.np.add_node(Camera())