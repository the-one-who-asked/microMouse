def sensor(direction: str) -> int:
    """Accesses a given sensor to find the distance between it and the nearest wall in the direction its pointing."""

    return 1


class Mouse:

    def __init__(self) -> None:
        """Instantiates a mouse object with the key methods and attributes needed to solve the maze."""

        self._x = 0
        self._y = 0
        self._orient = "n"
        self._walls = [[0 for _ in range(15)] for _ in range(15)]
        # The underscores prefacing each attribute indicate that they are private and should only be manipulated by the mouse's methods

    @property
    def x(self) -> int:
        return self._x
        # This method allows main to access x as a read-only attribute

    @property
    def y(self) -> int:
        return self._y
        # This method allows main to access y as a read-only attribute

    @property
    def orient(self) -> str:
        return self._orient
        # This method allows main to access orient as a read-only attribute

    @property
    def walls(self) -> list:
        return self._walls
        # This method allows main to access walls as a read-only attribute

    def scan(self, direction: str) -> bool:
        """Finds the distance of the nearest wall in a given direction."""

        direction = direction[0].lower()
        if direction not in ("l", "f", "r"):
            raise ValueError('direction must be either "left", "forward" or "right"')
        orient = ("n", "e", "s", "w")[("n", "e", "s", "w").index(self._orient) + {"l": -1, "r": 1}.get(direction, 0)]
        distance = sensor(direction)
        # Calculates the orientation of the selected sensor and scans for the nearest wall in that direction

        if orient == "n" and self._y + distance != 15:
            self._walls[self._y + distance][0] = 1
        elif orient == "e" and self._x + distance != 15:
            self._walls[0][self._x + distance] = 1
        elif orient == "s" and self._y - distance != 0:
            self._walls[self._y - distance][0] = 1
        if orient == "w" and self._x - distance != 0:
            self._walls[0][self._x - distance] = 1
        # Updates the mouse's memory to store the location of the wall found

        return distance == 0
        # Returns whether there is a wall directly in front of the sensor used

    def move(self) -> None:
        """Moves mouse 1 unit square (25 cm) forward."""

        if self._orient == "n":
            self._y += 1
        elif self._orient == "s":
            self._y -= 1
        elif self._orient == "e":
            self._x += 1
        else:
            self._y -= 1
        # Changes the mouse's coordinates with respect to its orientation

    def rotate(self, direction: str) -> None:
        """Rotates mouse 90º in the direction specified."""

        if direction[0].lower() == "l":
            self._orient = "w"
        elif direction[0].lower() == "r":
            self._orient = "e"
        else:
            raise ValueError('direction must be either "left" or "right"')
        # Changes the mouse's orientation with respect to the direction provided


def main():
    mouse = Mouse()


if __name__ == "__main__":
    main()
