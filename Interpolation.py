class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "x: " + str(self.x) + "y: " + str(self.y)


class LagrangeInterpolation:
    def __init__(self, points_list):
        self.points_list = points_list
        self.PX = []
        self.create_lagrange_interpolation()

    class SubFunction:

        def __init__(self, xk, xj):
            self.xk = xk
            self.xj = xj

        def __str__(self):
            return "(x - " + str(self.xk) + ") \\ (" + str(self.xj) + " - " + str(self.xk) + ")"

        def get_value(self, x):
            return (x - self.xk) / (self.xj - self.xk)

    def create_lagrange_interpolation(self):
        index1 = 0

        for points1 in self.points_list:
            temp = [points1.y]
            index2 = 0
            for points2 in self.points_list:
                if index1 != index2:
                    temp.append(self.SubFunction(points2.x, points1.x))
                index2 += 1
            self.PX.append(temp)
            index1 += 1

    def evaluate_PX_at_x(self, x):
        total = 0.0
        for PjX in self.PX:
            temp = PjX[0]
            for z in PjX[1:]:
                temp *= z.get_value(x)
            total += temp
        return total


def main():
    Points_list = [Point(0, 1), Point(-1, 0), Point(1, 1), Point(3, 2)]
    test = LagrangeInterpolation(Points_list)
    print(test.evaluate_PX_at_x(10))


if __name__ == "__main__":
    main()
