import argparse
from environment import Environment
from planner import BFS, UCS, AStar, LocalSearch

def run_planner(planner_name, map_filepath):
    """Initializes and runs a specified planner on a given map."""
    env = Environment(1, 1)  # Placeholder
    env.load_map(map_filepath)

    planner_classes = {
        'BFS': BFS,
        'UCS': UCS,
        'AStar': AStar,
        'LocalSearch': LocalSearch,
    }

    if planner_name not in planner_classes:
        print(f"Error: Planner '{planner_name}' not recognized. Choose from: {', '.join(planner_classes.keys())}")
        return

    planner = planner_classes[planner_name](env)
    print(f"Running {planner_name} on map '{map_filepath}'...")
    path = planner.find_path(env.start, env.goal)

    if path:
        print("\nPath found! Here is the grid with the path marked:")
        grid_copy = [list(row) for row in env.grid]
        for x, y in path:
            if (x, y) == env.start:
                grid_copy[y][x] = 'S'
            elif (x, y) == env.goal:
                grid_copy[y][x] = 'G'
            else:
                grid_copy[y][x] = 'P'
        
        for row in grid_copy:
            print("".join(str(cell) for cell in row))
    else:
        print("\nNo path could be found.")

def main():
    parser = argparse.ArgumentParser(description="Autonomous Delivery Agent CLI")
    parser.add_argument(
        '--planner', 
        type=str, 
        required=True, 
        choices=['BFS', 'UCS', 'AStar', 'LocalSearch'], 
        help="The pathfinding algorithm to use."
    )
    parser.add_argument(
        '--map',
        type=str,
        required=True,
        help="The path to the map file."
    )
    
    args = parser.parse_args()

    run_planner(args.planner, args.map)

if __name__ == "__main__":
    main()
