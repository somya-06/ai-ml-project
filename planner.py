import heapq
import random
import time

class Node:
    """Represents a node in the search tree."""
    def __init__(self, position, parent=None, cost=0, heuristic=0):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.f_cost = cost + heuristic

    def __lt__(self, other):
        """
        Comparison for priority queue (heapq).
        Nodes with lower f_cost are prioritized.
        """
        return self.f_cost < other.f_cost

class Planner:
    """
    Base class for all planning algorithms.
    Contains common methods for path reconstruction and logging.
    """
    def __init__(self, environment):
        self.env = environment
        self.start_time = None
        self.end_time = None
        self.nodes_expanded = 0
        self.path_cost = 0

    def find_path(self, start, goal):
        """Finds a path from start to goal. To be implemented by subclasses."""
        raise NotImplementedError

    def reconstruct_path(self, node):
        """Reconstructs the path from a goal node back to the start."""
        path = []
        current = node
        while current:
            path.append(current.position)
            current = current.parent
        return path[::-1]  # Reverse the path to get it from start to goal
    
    def log_results(self, path):
        """Logs the results of the planning run."""
        self.end_time = time.time()
        self.path_cost = sum(self.env.get_cost(pos) for pos in path)
        print("--- Planner Results ---")
        print(f"Algorithm: {self.__class__.__name__}")
        print(f"Path Cost: {self.path_cost}")
        print(f"Nodes Expanded: {self.nodes_expanded}")
        print(f"Time Taken: {self.end_time - self.start_time:.4f} seconds")
        print(f"Path Found: {path}")

class BFS(Planner):
    """
    Breadth-First Search (BFS) for pathfinding.
    
    Uninformed search that explores the grid level by level.
    It guarantees finding the shortest path in terms of the number of steps,
    but not necessarily the lowest-cost path.
    """
    def find_path(self, start, goal):
        self.start_time = time.time()
        queue = [Node(start)]
        visited = {start}
        self.nodes_expanded = 0
        
        while queue:
            self.nodes_expanded += 1
            current_node = queue.pop(0)

            if current_node.position == goal:
                path = self.reconstruct_path(current_node)
                self.log_results(path)
                return path

            for neighbor_pos in self.env.get_neighbors(current_node.position):
                if neighbor_pos not in visited and self.env.get_cost(neighbor_pos) != float('inf'):
                    visited.add(neighbor_pos)
                    queue.append(Node(neighbor_pos, parent=current_node))
        
        self.log_results([])
        return None  # No path found

class UCS(Planner):
    """
    Uniform-Cost Search (UCS) for pathfinding.
    
    Uninformed search that explores the grid by prioritizing paths with the lowest
    cumulative cost. It guarantees finding the lowest-cost path.
    """
    def find_path(self, start, goal):
        self.start_time = time.time()
        # Priority queue: stores tuples of (cost, node)
        p_queue = [(0, Node(start))]
        visited_costs = {start: 0}
        self.nodes_expanded = 0

        while p_queue:
            self.nodes_expanded += 1
            cost, current_node = heapq.heappop(p_queue)

            if current_node.position == goal:
                path = self.reconstruct_path(current_node)
                self.log_results(path)
                return path

            for neighbor_pos in self.env.get_neighbors(current_node.position):
                new_cost = cost + self.env.get_cost(neighbor_pos)
                if neighbor_pos not in visited_costs or new_cost < visited_costs[neighbor_pos]:
                    visited_costs[neighbor_pos] = new_cost
                    heapq.heappush(p_queue, (new_cost, Node(neighbor_pos, parent=current_node, cost=new_cost)))

        self.log_results([])
        return None

class AStar(Planner):
    """
    A* (A-Star) Search for pathfinding.
    
    Informed search that uses a heuristic to guide its search towards the goal.
    It finds the lowest-cost path and is generally much faster than UCS.
    The heuristic must be admissible (never overestimates the cost) to guarantee
    an optimal solution.
    """
    def manhattan_distance(self, pos, goal):
        """Admissible Manhattan distance heuristic."""
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def find_path(self, start, goal):
        self.start_time = time.time()
        p_queue = [Node(start, cost=0, heuristic=self.manhattan_distance(start, goal))]
        visited_costs = {start: 0}
        self.nodes_expanded = 0

        while p_queue:
            self.nodes_expanded += 1
            current_node = heapq.heappop(p_queue)
            
            if current_node.position == goal:
                path = self.reconstruct_path(current_node)
                self.log_results(path)
                return path

            for neighbor_pos in self.env.get_neighbors(current_node.position):
                new_cost = current_node.cost + self.env.get_cost(neighbor_pos)
                if neighbor_pos not in visited_costs or new_cost < visited_costs[neighbor_pos]:
                    visited_costs[neighbor_pos] = new_cost
                    new_node = Node(
                        neighbor_pos, 
                        parent=current_node, 
                        cost=new_cost, 
                        heuristic=self.manhattan_distance(neighbor_pos, goal)
                    )
                    heapq.heappush(p_queue, new_node)

        self.log_results([])
        return None

class LocalSearch(Planner):
    """
    Local Search (Hill-Climbing with Random Restarts) for replanning.
    
    This is not a traditional pathfinding algorithm; it's a heuristic-based
    replanning strategy. It's used when dynamic obstacles appear, and the agent
    needs to quickly find a new, good (not necessarily optimal) path.
    """
    def find_path(self, start, goal):
        self.start_time = time.time()
        
        # A simple hill-climbing implementation.
        # It takes small steps towards the goal. If it gets stuck, it restarts.
        path = [start]
        current_pos = start
        self.nodes_expanded = 0
        
        for _ in range(100): # Max iterations to prevent infinite loops
            self.nodes_expanded += 1
            if current_pos == goal:
                self.log_results(path)
                return path
            
            best_neighbor = None
            min_cost = float('inf')
            
            # Find the neighbor that gets us closest to the goal
            for neighbor_pos in self.env.get_neighbors(current_pos):
                if self.env.get_cost(neighbor_pos) == float('inf'):
                    continue
                
                # Simple heuristic: Manhattan distance to goal
                cost = self.env.get_cost(neighbor_pos) + abs(neighbor_pos[0] - goal[0]) + abs(neighbor_pos[1] - goal[1])
                
                # Simulate dynamic replanning here
                # Log a message to show the replan happening
                if self.env.is_collision(neighbor_pos, len(path)):
                    print(f"ALERT: Dynamic obstacle detected at {neighbor_pos}. REPLANNING!")
                    # The simple hill-climbing will just avoid this node.
                    continue

                if cost < min_cost:
                    min_cost = cost
                    best_neighbor = neighbor_pos
            
            if best_neighbor:
                path.append(best_neighbor)
                current_pos = best_neighbor
            else:
                # Get stuck, do a "random restart" by choosing a random neighbor
                valid_neighbors = [n for n in self.env.get_neighbors(current_pos) if self.env.get_cost(n) != float('inf')]
                if valid_neighbors:
                    current_pos = random.choice(valid_neighbors)
                    path.append(current_pos)
        
        print("ALERT: Local search failed to find a path within max iterations.")
        self.log_results([])
        return None




