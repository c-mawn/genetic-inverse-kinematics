import math


def bitstring_to_rad(thetas: list[list[int]], bits_per_theta: int = 16) -> list[float]:
    """
    converts the theta values from the bitstring repr to the radian repr

    args:
        thetas: list of lists of ints representing the binary values of each
            theta in the robot arm
    returns:
        theta_rad: list of floats representing a radian value for each theta
    """
    theta_rad = []
    for theta in thetas:
        bitstring = "".join(str(x) for x in theta)
        dec = int(bitstring, 2)
        ratio = dec / (2**bits_per_theta)
        theta_rad.append(ratio * 2 * math.pi)
    return theta_rad

    # return [(int("".join(theta), 2) / (2**self.bits_per_theta) * 2 * math.pi) for theta in thetas]
    # uncomment if ur cool
