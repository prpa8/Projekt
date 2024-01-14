import heapq

class CustomAStarSolver:
    def __init__(self, maze, start, end):
        self.maze = maze
        self.start = start
        self.end = end

    def custom_heuristic(self, position):
        return (abs(position[0] - self.end[0]) + abs(position[1] - self.end[1])) ** 0.5

    def find_neighbors(self, position):
        directions = self.get_directions()
        neighbors = []
        for dx, dy in directions:
            new_position = (position[0] + dx, position[1] + dy)
            if self.is_valid_position(new_position):
                neighbors.append(new_position)
        return neighbors

    def is_valid_position(self, position):
        return 0 <= position[0] < len(self.maze) and 0 <= position[1] < len(self.maze[0]) and self.maze[position[0]][position[1]] == 0

    def get_directions(self):
        return [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]

    def solve(self):
        open_set = []
        heapq.heappush(open_set, (0, self.start))
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.custom_heuristic(self.start)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == self.end:
                return self.reconstruct_path(came_from)

            for neighbor in self.find_neighbors(current):
                tentative_g_score = g_score[current] + self.step_cost(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.custom_heuristic(neighbor)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def step_cost(self, current, neighbor):
        return 1

    def reconstruct_path(self, came_from):
        path = []
        current = self.end
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path[::-1]

maze = [
[0, 0, 0, 0, 1],
[1, 1, 0, 0, 0],
[0, 0, 1, 0, 0],
[0, 0, 1, 1, 0],
[0, 0, 0, 0, 0]
]
start = (0, 0)
end = (4, 4)

solver = CustomAStarSolver(maze, start, end)
path = solver.solve()
print(path)