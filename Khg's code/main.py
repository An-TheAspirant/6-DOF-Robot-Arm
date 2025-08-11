from engine.loader import *
import engine.inverse as inv
a = np.array([1, 2, 3])

# inv.dh_transform(param)
print(a)


class Obj:
    def __init__(self, val):
        self.val = val

    def add(self, x):
        self.val += x

    def print_value(self):
        print("The value is", self.val)

Khang = Obj(1)
Khang.add(2)
Khang.print_value()

val = 0

def add_Khang(x):
    val += x
    print("The value is", val)


add()