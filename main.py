import numpy as np
import matplotlib.pyplot as plt


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

        file = open("1stworld.maz", "rb")
        b_maze = list(file.read())
        file.close()
        from_bin = lambda n: "".join([(n := n - val, orient)[1] for (orient, val) in [("w", 8), ("s", 4), ("e", 2), ("n", 1)] if n >= val])
        self.maze = [[from_bin(n) for n in b_maze[i: 256 + i: 16]] for i in range(16)]
        # This loads a virtual maze from a past competition which will be used for testing and simmulation

        self.sim_trail = [(0, 0)]
        # This variable will be used to track the coordinates of the mouse to more easily make a simulation of it

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

    def reset(self):
        """Resets the mouse to it's initial state in order to begin a run or another search."""

        self._x = 0
        self._y = 0
        self._orient = "n"
        self.sim_trail = []

    def scan(self, direction: str) -> bool:
        """Finds the distance of the nearest wall in a given direction."""

        direction = direction[0].lower()
        if direction not in ("l", "f", "r"):
            raise ValueError('direction must be either "left", "forward" or "right"')
        orient = ("n", "e", "s", "w")[(("n", "e", "s", "w").index(self._orient) + {"l": -1, "r": 1}.get(direction, 0)) % 4]
        wall = orient in self.maze[self.y][self.x]
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
            self._x -= 1
        self._trail.append(self._orient)
        self.sim_trail.append((self.x, self.y))
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
            neighbouring = [("n", (x, y-1)), ("s", (x, y+1)), ("e", (x-1, y)), ("w", (x+1, y))]
            sink.extend([coords for (neighbour, coords) in neighbouring if all(-1 < xy < 16 for xy in coords) and coords not in prev_source + source and neighbour not in walls[coords[1]][coords[0]]])
        # The sink variable is a list of all the coordinates which neighbour (without any walls in between) a current source

        for (x, y) in sink:
            floodmap[y][x] = i
        prev_source = source.copy()
        source = list(set(sink))
        # Each coordinate in the sink is given a value based on how many iterations it took to reach this coordinate.
        # This value is also inherently equal to the shortest path between a point and the original source: the destination

    return floodmap


def search(mouse: Mouse) -> None:
    """The search portion of the algorithm, in which the mouse explores the maze to hopefully find the optimal path."""

    while (mouse.x, mouse.y) not in [(x, y) for y in (7, 8) for x in (7, 8)]:
        # Initializes a loop to calculate the path of the mouse till it reaches the destination

        for i in ("l", "f", "r"):
            mouse.scan(i)
        # Scans for neighbouring walls to the left of, in front of and to the right of the mouse

        floodmap = floodfill(mouse.walls, mouse.x, mouse.y)
        # Generates a map showing the distance of most points (including the mouse) from the destination

        neighbouring = [("n", (mouse.x, mouse.y+1)), ("s", (mouse.x, mouse.y-1)), ("e", (mouse.x+1, mouse.y)), ("w", (mouse.x-1, mouse.y))]
        move_to = [orient for orient, (x, y) in neighbouring if -1 < x < 16 and -1 < y < 16 and floodmap[y][x] == floodmap[mouse.y][mouse.x] - 1 and orient not in mouse.walls[mouse.y][mouse.x]][0]
        turns = ("n", "e", "s", "w").index(move_to) - ("n", "e", "s", "w").index(mouse.orient)
        if turns == 3:
            turns = -1
        if turns == -3:
            turns = 1
        # Finds which direction the mouse needs to move in to get one square closer to the destination

        for _ in range(abs(turns)):
            mouse.rotate("l" if turns < 0 else "r")
        mouse.move()
        # Moves the mouse one square forward in the direction previously calculated


def run(mouse: Mouse) -> None:
    """The run portion of the algorithm, in which the mouse follows the optimal path found to get to the destination as fast as possible."""

    mouse.reset()
    # Reset's the mouse's coordinates and orient to the default

    long = True
    prev_trail = mouse.trail.copy()
    while long:
        # Initialises a loop which will end when there are no more neighbouring moves in the trail that contradict each other

        long = False
        trail = []
        include_next = True
        for pair in zip(prev_trail[:-1], prev_trail[1:]):
            if pair in (("n", "s"), ("s", "n"), ("w", "e"), ("e", "w")):
                long = True
                include_next = False
            elif include_next:
                trail.append(pair[0])
            else:
                include_next = True
        # Compares each neighbouring move in the trail to see if they contradict each other, removing pairs that do

        prev_trail = trail + [prev_trail[-1]]
    # Removes sections of the trail where the mouse hits a dead end and turns back around
    # This shortens the trail as much as possible while being 100% sure the trail will still lead from start to end

    for orient in trail + [mouse.trail[-1]]:
        # Follows the shortened trail from start to finish, completing the run

        turns = ("n", "e", "s", "w").index(orient) - ("n", "e", "s", "w").index(mouse.orient)
        if turns == 3:
            turns = -1
        if turns == -3:
            turns = 1
        # Finds which direction the mouse needs to move in to get one square closer to the destination

        for _ in range(abs(turns)):
            mouse.rotate("l" if turns < 0 else "r")
        mouse.move()
        # Moves the mouse one square forward in the direction previously calculated


def main():

    mouse = Mouse()
    # Initialises a mouse object containing all the methods that will be accessed by the physical mouse's hardware

    search(mouse)
    mouse.reset()
    run(mouse)
    # Searches for the shortest path from start to finish and then runs it

    file = open("100.txt", "r")
    maze = [[int(wall == " ") for wall in line[::2]] for line in file.read().split("\n")[::-1]]
    dists = np.tile([1.6, 0.4], (np.shape(maze)[0] + 1) // 2).cumsum()
    file.close()
    # Opens the text version of the maze to simmulate it using matplotlib

    plt.figure()
    plt.axes().set_aspect("equal")
    point = plt.plot(3, 3, "ro")
    for (x, y) in mouse.sim_trail:
        plt.pcolormesh(dists, dists, maze)
        point[0].remove()
        point = plt.plot(2*x + 3, 2*y + 3, "ro")
        plt.pause(0.05)
    plt.show()
    # The above code uses matplotlib to simmulate the movement of the mouse


if __name__ == "__main__":
    main()
