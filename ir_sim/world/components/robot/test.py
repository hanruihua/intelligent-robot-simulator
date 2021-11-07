from collections import namedtuple
import numpy as np

point = namedtuple('point', 'x y')

a = np.array([1, 2])

c = point(a[0], a[1])

print(c)