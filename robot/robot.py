from nodes import Node, NodeManager

class Robot():
    def __init__(self):
        self.nm = NodeManager()
        self.initNodes()
        self.nm.spin()

    def initNodes(self):
        self.nm.add_node(Brain())
        self.np.add_node(Camera())
        self.nm.add_node(Wheels())


if __name__ == '__main__':
    r = Robot()