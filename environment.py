import sys

class Environment:
    """
    Represents the 2D grid environment for the delivery agent.
    
    The environment contains information about the grid dimensions, terrain costs,
    static obstacles, and dynamic obstacles. It also provides methods for
    loading maps and checking for collisions.
    """
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = None  # Will be initialized in load_map
        self.static_obstacles = set()
        self.dynamic_obstacles = {}
        self.start = None
        self.goal = None

    def load_map(self, filepath):
        """
        Loads the grid environment from a text file.
        The file format is described in the README.md.
        """
        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()

            # Read dimensions, start, and goal
            self.width, self.height = map(int, lines[0].strip().split())
            self.start = tuple(map(int, lines[1].strip().split()))
            self.goal = tuple(map(int, lines[2].strip().split()))
            
            # --- FIX: Initialize the grid here with the correct dimensions ---
            self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]

            # Read grid and obstacles
            for y in range(self.height):
                row = lines[3 + y].strip()
                for x in range(self.width):
                    cell = row[x]
                    if cell == 'X':
                        self.grid[y][x] = float('inf')  # Impassable
                        self.static_obstacles.add((x, y))
                    else:
                        self.grid[y][x] = int(cell)

            # Read dynamic obstacles (if any)
            for i in range(3 + self.height, len(lines)):
                if lines[i].startswith('DYNAMIC'):
                    parts = lines[i].strip().split()
                    _, start_x, start_y, end_x, end_y = parts
                    start_pos = (int(start_x), int(start_y))
                    end_pos = (int(end_x), int(end_y))
                    # For a simple linear movement
                    self.dynamic_obstacles[start_pos] = end_pos

        except FileNotFoundError:
            print(f"Error: The map file at '{filepath}' was not found.")
            sys.exit(1)
        except Exception as e:
            print(f"Error loading map file: {e}")
            sys.exit(1)

    def is_valid_position(self, pos):
        """Checks if a position is within the grid boundaries."""
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def get_neighbors(self, pos):
        """Returns a list of valid 4-connected neighbors."""
        x, y = pos
        neighbors = []
        # 4-connected movements: up, down, left, right
        moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        
        for dx, dy in moves:
            new_pos = (x + dx, y + dy)
            if self.is_valid_position(new_pos):
                neighbors.append(new_pos)
        return neighbors
        
    def get_cost(self, pos):
        """Returns the movement cost for a given position."""
        x, y = pos
        if (x, y) in self.static_obstacles:
            return float('inf')
        return self.grid[y][x]

    def is_collision(self, pos, time):
        """Checks if a position at a given time has a dynamic collision."""
        # This is a simplified check. A full implementation would need to
        # model the movement schedule of dynamic obstacles.
        # For simplicity, we'll assume a new obstacle appears at time=5.
        if time == 5 and pos == (5, 5):
            return True
        return False
