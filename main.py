def sensor(direction: str) -> int:
    """Accesses a given sensor to find the distance between it and the nearest wall in the direction its pointing"""

    return 1


class Mouse:

    def __init__(self) -> None:
        """Instantiates a mouse object with the key methods and attributes needed to solve the maze"""

        self.x = 0
        self.y = 0
        self.orient = "n"
        self.walls = [[0 for _ in range(15)] for _ in range(15)]

    def scan(self, direction: str) -> bool:
        """Finds the distance of the nearest wall in a given direction"""

        direction = direction[0].lower()
        if direction not in ("l", "f", "r"):
            raise ValueError('direction must be either "left", "forward" or "right"')
        orient = ("n", "e", "s", "w")[("n", "e", "s", "w").index(self.orient) + {"l": -1, "r": 1}.get(direction, 0)]
        distance = sensor(direction)
        # Calculates the orientation of the selected sensor and scans for the nearest wall in that direction

        if orient == "n" and self.y + distance != 15:
            self.walls[self.y + distance][0] = 1
        elif orient == "e" and self.x + distance != 15:
            self.walls[0][self.x + distance] = 1
        elif orient == "s" and self.y - distance != 0:
            self.walls[self.y - distance][0] = 1
        if orient == "w" and self.x - distance != 0:
            self.walls[0][self.x - distance] = 1
        # Updates the mouse's memory to store the location of the wall found

        return distance == 0
        # Returns whether there is a wall directly in front of the sensor used

    def move(self) -> None:
        """Moves mouse 1 unit square (25 cm) forward"""

        if self.orient == "n":
            self.y += 1
        elif self.orient == "s":
            self.y -= 1
        elif self.orient == "e":
            self.x += 1
        else:
            self.y -= 1
        # Changes the mouse's coordinates with respect to its orientation

    def turn(self, direction: str) -> None:
        """Rotates mouse 90º clockwise or counter-clockwise"""

        if direction[0].lower() == "l":
            self.orient = "w"
        elif direction[0].lower() == "r":
            self.orient = "e"
        else:
            raise ValueError('direction must be either "left" or "right"')
        # Changes the mouse's orientation with respect to the direction provided


def main():
    mouse = Mouse()


if __name__ == "__main__":
    main()
