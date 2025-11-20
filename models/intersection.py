class Intersection:
    def __init__(self, intersection_id, x, y, z):
        self.id = intersection_id
        self.x = x
        self.y = y
        self.z = z

    @property
    def coords(self):
        return (self.x, self.y, self.z)