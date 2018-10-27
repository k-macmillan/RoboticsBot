from nodes import Node, NodeManager


class Robot():
    def __init__(self):
        self.verbose = False
        self.nm = NodeManager()
        self.initNodes()
        self.nm.spin()

    def initNodes(self):
        self.nm.add_node(Brain(verbose=self.verbose))
        self.np.add_node(Camera())
        self.nm.add_node(Wheels(verbose=self.verbose))


if __name__ == '__main__':
    r = Robot()