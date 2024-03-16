import numpy as np
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
        self._walls = [["esw"] + ["w"] + [""] * 14] + [[""] * 16 for _ in range(15)]
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
        self._orient = ("n", "e", "s", "w")[(("n", "e", "s", "w").index(self._orient) + {"l": -1, "r": 1}[direction[0].lower()]) % 4]

        # Applies rotational scanning to the first quadrant
        if not ((direction == "l" and self.orient == "s") or (direction == "r" and self.orient == "w")):
            quadrant = scan([row[self.x:] for row in self.maze[self.y:]], ("n", "e", "s", "w"), 16 - self.x)
            self._walls = self.walls[:self.y] + [a[:self.x] + b for a, b in zip(self.walls, quadrant)]

        # Scans the second quadrant
        if not ((direction == "l" and self.orient == "w") or (direction == "r" and self.orient == "n")):
            quadrant = scan([row[self.x:] for row in self.maze[:self.y + 1][::-1]], ("s", "e", "n", "w"), 16 - self.x)[::-1]
            self._walls = [a[:self.x] + b for a, b in zip(self.walls, quadrant)] + self.walls[self.y + 1:]

        # Scans the third quadrant
        if not ((direction == "l" and self.orient == "n") or (direction == "r" and self.orient == "e")):
            quadrant = [row[::-1] for row in scan([row[:self.x + 1][::-1] for row in self.maze[:self.y + 1][::-1]], ("s", "w", "n", "e"), self.x + 1)[::-1]]
            self._walls = [b + a[self.x + 1:] for a, b in zip(self.walls, quadrant)] + self.walls[self.y + 1:]

        # Scans the fourth quadrant
        if not ((direction == "l" and self.orient == "e") or (direction == "r" and self.orient == "s")):
            quadrant = [row[::-1] for row in scan([row[:self.x + 1][::-1] for row in self.maze[self.y:]], ("n", "w", "s", "e"), self.x + 1)]
            self._walls = self.walls[:self.y] + [b + a[self.x + 1:] for a, b in zip(self.walls, quadrant)]
        print(self.walls)


def add_walls(walls1: List[List[str]], walls2: List[List[str]]) -> List[List[str]]:
    return [[a + "".join(i for i in b if i not in a) for a, b in zip(c, d)] for c, d in zip(walls1, walls2)]


def scan(quadrant: List[List[str]], compass: Tuple[str, str, str, str], width: int, row: int = 0, start: int = 0) -> List[List[str]]:
    corridor = quadrant[0]
    quadrant_info = [[""] * width for _ in range(len(quadrant))]
    dead_end = False
    for i, square in enumerate(corridor):
        if not dead_end:
            if compass[0] in square:
                quadrant_info[0][i] += compass[0]
                if len(quadrant) != 1:
                    quadrant_info[1][start + i] += compass[2]
            elif len(quadrant) != 1:
                next_start = start + i + int((start + i - 0.5) // (row + 0.5))
                if next_start < 0:
                    next_start = 0
                high_gradient = False
                if next_start >= width:
                    high_gradient = True
                blocks = [j + i for j, square_ in enumerate(quadrant[1][i: (width if high_gradient else next_start)]) if compass[1] in square_]
                if blocks:
                    quadrant_info[1][blocks[0]] += compass[1]
                    if not high_gradient:
                        quadrant_info[1][blocks[0] + 1] += compass[3]
                else:
                    next_end = start + i + 1 + int((start + i + 0.5) // (row + 0.5))
                    tip = False
                    if next_end > width:
                        next_end = width - 1
                        tip = True
                    peek = [quadrant[1][next_start: next_end]]
                    if not tip:
                        peek[0][-1].replace(compass[1], "")
                    if len(quadrant) != 2:
                        peek += quadrant[2:]
                    quadrant_info = [quadrant_info[0]] + add_walls(scan(peek, compass, width, row + 1, next_start), quadrant_info[1:])
            if compass[1] in square:
                quadrant_info[0][i] += compass[1]
                if start + i + 1 != width:
                    quadrant_info[0][i + 1] += compass[3]
                dead_end = True
    return quadrant_info


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
                    elif coords not in prev_source + sink:
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

