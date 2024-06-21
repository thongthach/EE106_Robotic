#!/usr/bin/env python3

from heapq import heappop, heappush

def neighbors(current):
    # Define the list of 4 neighbors
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    return [(current[0] + nbr[0], current[1] + nbr[1]) for nbr in neighbors]

def heuristic_distance(candidate, goal):
    return abs(candidate[0] - goal[0]) + abs(candidate[1] - goal[1])

def get_path_from_A_star(start, goal, obstacles):
    open_list = [(0, start)]
    past_cost = {}
    past_cost[start] = 0
    parent = {}
    parent[start] = None
    closed_list = set()

    while open_list:
        current = open_list.pop(0)[1]
        closed_list.add(current)

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            
            path.reverse()
            return path[1:]


        for candidate in neighbors(current):
            if candidate in obstacles or candidate in closed_list:
                print("Skip this iteration")
                continue

            tentative_past_cost = past_cost[current] + 1  # Cost of moving to the neighbor is always 1

            if candidate not in past_cost or tentative_past_cost < past_cost[candidate]:
                past_cost[candidate] = tentative_past_cost
                parent[candidate] = current
                new_cost = tentative_past_cost + heuristic_distance(candidate, goal)
                heappush(open_list, (new_cost, candidate))

    return None  # No path found

if __name__ == '__main__':
    start = (0, 0)
    goal = (-5, -2)
    obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
    path = get_path_from_A_star(start, goal, obstacles)
    if path:
        print("Path found:", path)
    else:
        print("No path found")

    # For the test case
    start = (0, 0)
    goal = (3, 3)
    obstacles = []
    path = get_path_from_A_star(start, goal, obstacles)
    if path:
        print("Path found:", path)
    else:
        print("No path found")

