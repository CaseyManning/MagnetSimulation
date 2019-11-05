import math
import numpy as np

def vec2angle(vec):
    x = vec[0]
    y = vec[1]
    # z = vec[2]
    # ax = math.atan(z/y)
    # ay = math.atan(x/z)

    if x == 0:
        if y <= 0:
            az = -np.pi/2
        else:
            az = np.pi/2
    else:
        az = math.atan2(y, x)
    return 0, 0, az


if __name__ == "__main__":
    print(180/3.14159 * np.array(vec2angle([0.11732139, 0, 0])))