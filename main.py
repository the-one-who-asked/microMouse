import matplotlib.pyplot as plt
from typing import Tuple, List


class Mouse:

    def __init__(self) -> None:
        """Instantiates a mouse object with the key methods and attributes which will, in the actual algorithm, bridge
        the hardware in the mouse to the software."""

        # The underscores prefacing each attribute indicate that they are private and should only be manipulated by the mouse's methods
        self._x = 0
        self._y = 0
        self._orient = "n"
        self._walls = [["" for _ in range(16)] for _ in range(16)]
        self._trail = []

        # This loads a virtual maze from a past competition which will be used for testing and simmulation
        file = open("1stworld.maz", "rb")
        b_maze = list(file.read())
        file.close()
        from_bin = lambda n: "".join([(n := n - val, orient)[1] for (orient, val) in [("w", 8), ("s", 4), ("e", 2), ("n", 1)] if n >= val])
        self.maze = [[from_bin(n) for n in b_maze[i: 256 + i: 16]] for i in range(16)]

    # This method allows main to access x as a read-only attribute
    @property
    def x(self) -> int:
        return self._x

    # This method allows main to access y as a read-only attribute
    @property
    def y(self) -> int:
        return self._y

    # This method allows main to access orient as a read-only attribute
    @property
    def orient(self) -> str:
        return self._orient

    # This method allows main to access walls as a read-only attribute
    @property
    def walls(self) -> list:
        return self._walls

    # This method allows main to access trail as a read-only attribute
    @property
    def trail(self) -> list:
        return self._trail

    def scan(self) -> None:
        """Updates walls by taking input from IR sensors"""

        # Updates northwards wall closest to mouse if a sensor is pointing in that direction
        if self.orient != "s":

            # Determines distance between mouse and this wall
            distance = [i for i, y in enumerate([row[self.x] for row in self.maze[self.y:]]) if "n" in y][0]
            # In the physical run, this line would be:
            # distance = round(readIR({"w": "l", "n": "f", "e": "r"}[self.orient]))

            # Updates the square with the north wall found
            if "n" not in self.walls[self.y + distance][self.x]:
                self._walls[self.y + distance][self.x] += "n"

            # Updates the square north to the square with the north wall found as the north wall would be a south wall for this square
            if self.y + distance != 15 and "s" not in self.walls[self.y + distance + 1][self.x]:
                self._walls[self.y + distance + 1][self.x] += "s"

        # Repeats the process for the sensor pointing to the north
        if self.orient != "w":
            distance = [i for i, x in enumerate(self.maze[self.y][self.x:]) if "e" in x][0]
            # In the physical run, this line would be:
            # distance = round(readIR({"n": "l", "e": "f", "s": "r"}[self.orient]))

            if "e" not in self.walls[self.y][self.x + distance]:
                self._walls[self.y][self.x + distance] += "e"
            if self.x + distance != 15 and "w" not in self.walls[self.y][self.x + distance + 1]:
                self._walls[self.y][self.x + distance + 1] += "w"

        # Repeats the process for the sensor pointing to the west
        if self.orient != "e":
            distance = [i for i, x in enumerate(self.maze[self.y][:self.x + 1]) if "w" in x][-1]
            # In the physical run, this line would be:
            # distance = round(readIR({"s": "l", "w": "f", "n": "r"}[self.orient]))

            if "w" not in self.walls[self.y][distance]:
                self._walls[self.y][distance] += "w"
            if distance != 0 and "e" not in self.walls[self.y][distance - 1]:
                self._walls[self.y][distance - 1] += "e"

        # Repeats the process for the sensor pointing to the south
        if self.orient != "n":
            distance = [i for i, y in enumerate([row[self.x] for row in self.maze[:self.y + 1]]) if "s" in y][-1]
            # In the physical run, this line would be:
            # distance = round(readIR({"e": "l", "s": "f", "w": "r"}[self.orient]))

            if "s" not in self.walls[distance][self.x]:
                self._walls[distance][self.x] += "s"
            if distance != 0 and "n" not in self.walls[distance - 1][self.x]:
                self._walls[distance - 1][self.x] += "n"

    def test_scan(self) -> None:
        square = self.maze[self.y][self.x]
        self.walls[self.y][self.x] = square
        if "n" in square and self.y != 15 and "s" not in self.walls[self.y + 1][self.x]:
            self.walls[self.y + 1][self.x] += "s"
        if "e" in square and self.x != 15 and "w" not in self.walls[self.y][self.x + 1]:
            self.walls[self.y][self.x + 1] += "w"
        if "s" in square and self.y != 0 and "n" not in self.walls[self.y - 1][self.x]:
            self.walls[self.y - 1][self.x] += "n"
        if "w" in square and self.x != 0 and "e" not in self.walls[self.y][self.x - 1]:
            self.walls[self.y + 1][self.x] += "e"

    def move(self) -> None:
        """Moves mouse 1 unit square forward."""

        # Changes the mouse's coordinates with respect to its orientation and adds movement to the mouse's trail
        if self._orient == "n":
            self._y += 1
        elif self._orient == "s":
            self._y -= 1
        elif self._orient == "e":
            self._x += 1
        else:
            self._x -= 1
        self._trail.append((self.x, self.y))

    def rotate(self, direction: str) -> None:
        """Rotates mouse 90º in the direction specified, scanning walls during this rotation"""

        # Changes the mouse's orientation with respect to the direction provided
        if direction not in ("l", "r"):
            raise ValueError('direction must be either "left" or "right"')
        self._orient = ("n", "e", "s", "w")[(("n", "e", "s", "w").index(self._orient) + {"l": -1, "r": 1}[direction[0].lower()]) % 4]


def floodfill(walls: list, mouse_x: int, mouse_y: int, destination: List[Tuple[int, int]], orientation: str, first_run: bool) -> Tuple[int, int]:
    """Calculates the shortest path between the mouse and the destination, assuming all walls in the maze are provided
    in the walls parameter."""

    # Source is the row of nodes in the breadth-first search through the coordinates which is currently being iterated through
    # Prev source is the previous row, used to make sure there are no repeats of coordinates in the search
    source = destination
    prev_source = source.copy()

    # Iterates through each row of nodes in the breadth-first search
    while True:

        # Allows for a pause between the return of the source leading to the mouse to check if there are any other sources which allow for rotation
        if first_run:
            mouse_sources = []

        # Sink is the next row of nodes, i.e. the coordinates which neighbour each coordinate in the source without a wall in between
        sink = []
        for (x, y) in source:

            # Iterates through the neighbours of each source to find potential sinks
            neighbouring = [("n", (x, y-1)), ("s", (x, y+1)), ("e", (x-1, y)), ("w", (x+1, y))]
            for direction, coords in neighbouring:

                # Checks for walls between neighbours and each source
                if all(0 <= coords[i] < 16 for i in (0, 1)) and (direction not in walls[coords[1]][coords[0]]):

                    # Finds sources leading to the mouse, returning them if they involve a rotation
                    if (mouse_x, mouse_y) == coords:
                        if first_run and neighbouring[neighbouring.index((direction, coords)) - 2][0] == orientation:
                            mouse_sources.append(direction)
                        else:
                            return direction

                    # Ensures there are no duplicates of each potential sink in the search
                    elif coords not in prev_source + source + sink:
                        sink.append(coords)

        # If the mouse is reached only from a source involving no rotations, returns this source
        if first_run and mouse_sources:
            return mouse_sources[0]

        # Aligns each array with the next row of nodes for the next iteration
        prev_source = source.copy()
        source = sink.copy()


def journey(mouse: Mouse, first_run: bool, run: bool) -> None:
    """The search portion of the algorithm, in which the mouse explores the maze to hopefully find the optimal path."""

    # Decides destination based on whether the mouse is heading to the goal or the starting square
    if run:
        destination = [(x, y) for y in (7, 8) for x in (7, 8)]
    else:
        destination = [(0, 0)]

    # Initializes a loop to calculate the path of the mouse till it reaches the destination
    while (mouse.x, mouse.y) not in destination:

        # Scans for neighbouring walls to the left of, in front of and to the right of the mouse
        mouse.scan()

        # Generates a map showing the distance of most points (including the mouse) from the destination
        direction = floodfill(mouse.walls, mouse.x, mouse.y, destination, mouse.orient, first_run)

        # Finds which direction the mouse needs to move in to get one square closer to the destination
        turns = ("n", "e", "s", "w").index(direction) - ("n", "e", "s", "w").index(mouse.orient)
        turns = {3: -1, -3: 1}.get(turns, turns)

        # Moves the mouse one square forward in the direction previously calculated
        for _ in range(abs(turns)):
            mouse.rotate("l" if turns < 0 else "r")
        mouse.move()


def main():

    # Initialises a mouse object containing all the methods that will be accessed by the physical mouse's hardware
    mouse = Mouse()

    # Traces out the path of the mouse over 3 runs
    journey(mouse, True, True)
    journey(mouse, False, False)
    journey(mouse, False, True)
    journey(mouse, False, False)
    journey(mouse, False, True)

    # Opens the text version of the maze to simmulate it using matplotlib
    file = open("100.txt", "r")
    maze = [[int(wall == " ") for wall in line[::2]] for line in file.read().split("\n")[::-1]]
    dists = np.tile([1.6, 0.4], (np.shape(maze)[0] + 1) // 2).cumsum()
    file.close()

    # Uses matplotlib to simmulate the movement of the mouse
    plt.figure()
    plt.axis("equal")
    plt.axis("off")
    plt.pcolormesh(dists, dists, maze)
    point = plt.plot(3, 3, "ro")
    plt.pause(0.1)
    for (x, y) in mouse.trail:
        point[0].remove()
        point = plt.plot(2*x + 3, 2*y + 3, "ro")
        plt.pause(0.1)
    plt.show()


if __name__ == "__main__":
    main()
