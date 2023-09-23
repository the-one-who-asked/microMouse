def sensor(direction: str) -> int:
    """Accesses a given sensor to check if there is a neighbouring wall in the direction its pointing."""

    return False

def pretty_print(arr: list) -> None:
    """A debugging tool I can use to better visualise floodmaps and walls."""

    print("\n\n".join("  ".join(str(i).ljust(4) for i in row) for row in arr) + "\n" * 8)


class Mouse:

    def __init__(self) -> None:
        """Instantiates a mouse object with the key methods and attributes which will, in the actual algorithm,
        bridge the hardware in the mouse to the software."""

        self._x = 0
        self._y = 0
        self._orient = "n"
        self._walls = [["" for _ in range(16)] for _ in range(16)]
        self._trail = []
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

    @property
    def trail(self) -> list:
        return self._trail
        # This method allows main to access trail as a read-only attribute

    def scan(self, direction: str) -> bool:
        """Finds the distance of the nearest wall in a given direction."""

        direction = direction[0].lower()
        if direction not in ("l", "f", "r"):
            raise ValueError('direction must be either "left", "forward" or "right"')
        orient = ("n", "e", "s", "w")[(("n", "e", "s", "w").index(self._orient) + {"l": -1, "r": 1}.get(direction, 0)) % 4]
        wall = sensor(direction)
        # Calculates the orientation of the selected sensor and if there is a neighbouring wall in that direction

        if wall:
            self._walls[self._y][self._x] += orient
        # Updates the mouse's memory to store the location and orientation of the wall if one is found

        return wall
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
        self._trail.append(self._orient)
        # Changes the mouse's coordinates with respect to its orientation and adds movement to the mouse's trail

    def rotate(self, direction: str) -> None:
        """Rotates mouse 90º in the direction specified."""

        if direction not in ("l", "r"):
            raise ValueError('direction must be either "left" or "right"')
        self._orient = ("n", "e", "s", "w")[(("n", "e", "s", "w").index(self._orient) + {"l": -1, "r": 1}[direction[0].lower()]) % 4]
        # Changes the mouse's orientation with respect to the direction provided


def floodfill(walls: list, mouse_x: int, mouse_y: int) -> list:
    """Calculates the shortest distance between each coordinate and the destination,
    assuming all walls in the maze are provided in the walls parameter."""

    floodmap = [[0 for _ in range(16)] for _ in range(16)]
    source = [(x, y) for y in (7, 8) for x in (7, 8)]
    prev_source = source.copy()
    i = 0
    # Declares the necessary variables for the floodfill calculation

    while (mouse_x, mouse_y) not in source + prev_source or i < 2:
        # Initializes a loop to calculate the distance of each point from the destination till the distance from the mouse has been calculated

        i += 1
        sink = []
        for (x, y) in source:
            neighbouring = [("n", (x, y+1)), ("s", (x, y-1)), ("e", (x+1, y)), ("w", (x-1, y))]
            sink.extend([coords for (neighbour, coords) in neighbouring if neighbour not in walls[y][x] and all(-1 < i < 16 for i in coords) and coords not in prev_source + source])
        # The sink variable is a list of all the coordinates which neighbour (without any walls in between) a current source

        for (x, y) in sink:
            floodmap[y][x] = i
        prev_source = source.copy()
        source = list(set(sink))
        # Each coordinate in the sink is given a value based on how many iterations it took to reach this coordinate.
        # This value is also inherently equal to the shortest path between a point and the original source: the destination

    return floodmap


def main():

    mouse = Mouse()
    # Initializes the mouse object, containing all the methods which will in the final project be directly connected to the hardware

    while (mouse.x, mouse.y) not in [(x, y) for y in (7, 8) for x in (7, 8)]:
        # Initializes a loop to calculate the path of the mouse till it reaches the destination

        for i in ("l", "f", "r"):
            mouse.scan(i)
        # Scans for neighbouring walls to the left of, in front of and to the right of the mouse

        floodmap = floodfill(mouse.walls, mouse.x, mouse.y)
        # Generates a map showing the distance of most points (including the mouse) from the destination

        neighbouring = [("n", (mouse.x, mouse.y+1)), ("s", (mouse.x, mouse.y-1)), ("e", (mouse.x+1, mouse.y)), ("w", (mouse.x-1, mouse.y))]
        move_to = [orient for orient, (x, y) in neighbouring if floodmap[y][x] == floodmap[mouse.y][mouse.x] - 1][0]
        turns = ("n", "e", "s", "w").index(move_to) - ("n", "e", "s", "w").index(mouse.orient)
        if turns == 3:
            trurns = -1
        # Finds which direction the mouse needs to move in to get one square closer to the destination

        for _ in range(abs(turns)):
            mouse.rotate("l" if turns < 0 else "r")
        mouse.move()
        # Moves the mouse one square forward in the direction previously calculated

    print(mouse.trail)


if __name__ == "__main__":
    main()
