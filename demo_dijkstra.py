import heapq
from math import sqrt

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

class Polygon:
    def __init__(self, vertices):
        self.vertices = vertices

    def is_inside(self, point):
        n = len(self.vertices)
        inside = False
        p1x, p1y = self.vertices[0].x, self.vertices[0].y
        for i in range(n+1):
            p2x, p2y = self.vertices[i % n].x, self.vertices[i % n].y
            if point.y > min(p1y, p2y):
                if point.y <= max(p1y, p2y):
                    if point.x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (point.y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or point.x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

class Map:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.polygons = []

    def add_polygon(self, polygon):
        self.polygons.append(polygon)

    def is_valid_point(self, point):
        return 0 <= point.x <= self.width and 0 <= point.y <= self.height

    def is_valid_move(self, point1, point2):
        for polygon in self.polygons:
            if polygon.is_inside(point1) or polygon.is_inside(point2):
                return False
        return True

    def dijkstra(self, start, goal):
        visited = set()
        distances = {start: 0}
        pq = [(0, start)]

        while pq:
            current_distance, current_vertex = heapq.heappop(pq)

            if current_vertex in visited:
                continue

            visited.add(current_vertex)

            if current_vertex == goal:
                return distances[current_vertex]

            for neighbor in self.get_neighbors(current_vertex):
                if neighbor in visited:
                    continue

                new_distance = distances[current_vertex] + self.distance(current_vertex, neighbor)

                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    heapq.heappush(pq, (new_distance, neighbor))

        return None

    def get_neighbors(self, point):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = Point(point.x + dx, point.y + dy)
                if self.is_valid_point(neighbor) and self.is_valid_move(point, neighbor):
                    neighbors.append(neighbor)
        return neighbors

    def distance(self, point1, point2):
        return sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)




#test case
# doc du lieu
width, height = map(int, input("Enter map size: ").split(','))
start_x, start_y, goal_x, goal_y = map(int, input("Enter start and goal points: ").split(','))

# tao map
map = Map(width, height)

# so chuong ngai vat
num_polygons = int(input("Enter the number of polygons: "))

# tao chuong ngai vat
for i in range(num_polygons):
    polygon_data = input("Enter polygon vertices: ").split(',')
    vertices = [Point(int(polygon_data[j]), int(polygon_data[j+1])) for j in range(0, len(polygon_data), 2)]
    polygon = Polygon(vertices)
    map.add_polygon(polygon)

# tim duong ngan nhat
start = Point(start_x, start_y)
goal = Point(goal_x, goal_y)
shortest_distance = map.dijkstra(start, goal)
if shortest_distance is not None:
    print("Shortest distance:", shortest_distance)
else:
    print("No solution")
