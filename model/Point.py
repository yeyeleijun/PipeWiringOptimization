class Node:
    @staticmethod
    def change_point(p1: tuple, p2: tuple):
        # Manhattan distance between two arbitrary points
        distance = tuple(abs(x - y) for x, y in zip(p1, p2))
        count = distance.count(1)
        return count == 2

    def __init__(self, coord, parent=None, energy=None):
        self.coord = coord
        self.parent = parent

        if self.parent is None:
            self.depth = 1
            self.n_cp = 0
            self.energy = energy
        else:
            self.depth = self.parent.depth + 1
            self.energy = self.parent.energy + energy
            self.n_cp = self.parent.n_cp
            if self.parent.parent is not None:
                if self.change_point(self.parent.parent.coord, self.coord):
                    self.n_cp = self.parent.n_cp + 1

    def __lt__(self, other):
        return self.coord[0] < other.coord[0]

    def __eq__(self, other):
        return self.coord == other.coord