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


def get_t(x, xk, xk1):
    return (x - xk) / (xk1 - xk)


class CubicSplineInterpolation:
    def __init__(self, points, slopes_list):
        self.points = points
        self.slopes_list = slopes_list

    def H00(self, x):
        xk = self.points[0].x
        xk1 = self.points[1].x
        pk = self.points[0].y
        t = get_t(x, xk, xk1)
        return pk * (2 * pow(t, 3) - 3 * pow(t, 2) + 1)

    def H01(self, x):
        xk = self.points[0].x
        xk1 = self.points[1].x
        pk1 = self.points[1].y
        t = get_t(x, xk, xk1)
        return pk1 * (-2 * pow(t, 3) + 3 * pow(t, 2))

    def H10(self, x):
        xk = self.points[0].x
        xk1 = self.points[1].x
        mk = self.slopes_list[0]
        t = get_t(x, xk, xk1)
        return mk * (xk1 - xk) * (pow(t, 3) - 2 * pow(t, 2) + t)

    def H11(self, x):
        xk = self.points[0].x
        xk1 = self.points[1].x
        mk1 = self.slopes_list[1]
        t = get_t(x, xk, xk1)
        return mk1 * (xk1 - xk) * (pow(t, 3) - 1 * pow(t, 2))

    def get_value(self, x):
        return self.H00(x) + self.H01(x) + self.H10(x) + self.H11(x)


"""def main():
    points = [Point(142, 433), Point(37, 336)]
    slopes = [0, 0]
    test = CubicSplineInterpolation(points, slopes)
    print(test.get_value(140))


if __name__ == "__main__":
    main()"""
