# import heapq
# import cv2
# import matplotlib.pyplot as plt

# # Class Maze
# class Maze:
#     def __init__(self, roads_img_path):
#         (thresh, self.roads) = cv2.threshold(cv2.imread(roads_img_path, cv2.IMREAD_GRAYSCALE), 127, 255, cv2.THRESH_BINARY)
#         self.display_img = cv2.cvtColor(self.roads.copy(), cv2.COLOR_GRAY2RGB)

#     def bresenham_line(self, x0, y0, x1, y1):
#         points = []
#         dx = abs(x1 - x0)
#         dy = abs(y1 - y0)
#         sx = 1 if x0 < x1 else -1
#         sy = 1 if y0 < y1 else -1
#         err = dx - dy

#         while True:
#             points.append((x0, y0))
#             if x0 == x1 and y0 == y1:
#                 break
#             e2 = err * 2
#             if e2 > -dy:
#                 err -= dy
#                 x0 += sx
#             if e2 < dx:
#                 err += dx
#                 y0 += sy
                
#         return points

#     def isPointInObstacles(self, xCoord, yCoord):
#         is_point_in_obs = False
#         if self.roads[yCoord, xCoord] == 0:
#             is_point_in_obs = True
#         return is_point_in_obs

#     def isValidPoint(self, parentNodeX, parentNodeY, childNodeX, childNodeY):
#         valid_point = True
#         line_points = self.bresenham_line(parentNodeX, parentNodeY, childNodeX, childNodeY)
#         for x, y in line_points:
#             if self.isPointInObstacles(x, y):
#                 valid_point = False
#         return valid_point

#     def visualize_path(self, path):
#         for i in range(len(path) - 1):
#             cv2.line(self.display_img, path[i], path[i + 1], (255, 0, 0), 2) 
#         cv2.imshow("Planned Path", self.display_img)
#         cv2.waitKey(0)

#     def point_transform(self, pnt):
#         gazebo_x = (-1 * pnt[1] / 13.002) + 35.7567 + 0.01475
#         gazebo_y = (-1 * pnt[0] / 13.002) + 22.6746 + 0.01475
#         return (gazebo_x, gazebo_y)
        
#     def heuristic(self, p1, p2):
#         return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

#     def astar(self, start, goal):
#         open_list = []
#         heapq.heappush(open_list, (0, start))
#         came_from = {}
#         g_score = {start: 0}
#         f_score = {start: self.heuristic(start, goal)}

#         while open_list:
#             current = heapq.heappop(open_list)[1]
#             print(f"Current node: {current}")

#             if current == goal:
#                 path = []
#                 while current in came_from:
#                     path.append(current)
#                     current = came_from[current]
#                 path.append(start)
#                 return path[::-1]

#             neighbors = [
#                 (current[0] + 1, current[1]), (current[0] - 1, current[1]),
#                 (current[0], current[1] + 1), (current[0], current[1] - 1)
#             ]

#             for neighbor in neighbors:
#                 if 0 <= neighbor[0] < self.roads.shape[1] and 0 <= neighbor[1] < self.roads.shape[0]:
#                     if self.isPointInObstacles(neighbor[0], neighbor[1]):
#                         continue

#                     tentative_g_score = g_score[current] + 1

#                     if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                         came_from[neighbor] = current
#                         g_score[neighbor] = tentative_g_score
#                         f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
#                         heapq.heappush(open_list, (f_score[neighbor], neighbor))
#                         print(f"Adding neighbor to open list: {neighbor}")

#         return []

# # Path to the roads image
# roads_img_path = "roads.png"

# # Creating maze object
# maze = Maze(roads_img_path)

# # Starting point and goal point
# start_point = (98, 290)
# goal_point = (190, 326)

# # Ensure the start and goal points are not in obstacles
# if maze.isPointInObstacles(start_point[0], start_point[1]):
#     print("Start point is in an obstacle.")
# if maze.isPointInObstacles(goal_point[0], goal_point[1]):
#     print("Goal point is in an obstacle.")

# # Find the path using A* algorithm
# path = maze.astar(start_point, goal_point)
# print(path)

# if not path:
#     print("No path found.")

import heapq
import cv2
import matplotlib.pyplot as plt

# Class Maze
class Maze:
    def __init__(self, roads_img_path):
        (thresh, self.roads) = cv2.threshold(cv2.imread(roads_img_path, cv2.IMREAD_GRAYSCALE), 127, 255, cv2.THRESH_BINARY)
        self.display_img = cv2.cvtColor(self.roads.copy(), cv2.COLOR_GRAY2RGB)

    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return points

    def isPointInObstacles(self, xCoord, yCoord):
        is_point_in_obs = False
        if self.roads[yCoord, xCoord] == 0:
            is_point_in_obs = True
        return is_point_in_obs

    def isValidPoint(self, parentNodeX, parentNodeY, childNodeX, childNodeY):
        valid_point = True
        line_points = self.bresenham_line(parentNodeX, parentNodeY, childNodeX, childNodeY)
        for x, y in line_points:
            if self.isPointInObstacles(x, y):
                valid_point = False
        return valid_point

    def visualize_path(self, path):
        for i in range(len(path) - 1):
            cv2.line(self.display_img, path[i], path[i + 1], (255, 0, 0), 2) 
        cv2.imshow("Planned Path", self.display_img)
        cv2.waitKey(0)

    def point_transform(self, pnt):
        gazebo_x = (-1 * pnt[1] / 13.002) + 35.7567 + 0.01475
        gazebo_y = (-1 * pnt[0] / 13.002) + 22.6746 + 0.01475
        return (gazebo_x, gazebo_y)

# Class AStar
class AStar:
    def __init__(self, maze):
        self.maze = maze

    def heuristic(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    def astar(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current = heapq.heappop(open_list)[1]
            print(f"Current node: {current}")

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            neighbors = [
                (current[0] + 1, current[1]), (current[0] - 1, current[1]),
                (current[0], current[1] + 1), (current[0], current[1] - 1)
            ]

            for neighbor in neighbors:
                if 0 <= neighbor[0] < self.maze.roads.shape[1] and 0 <= neighbor[1] < self.maze.roads.shape[0]:
                    if self.maze.isPointInObstacles(neighbor[0], neighbor[1]):
                        continue

                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        print(f"Adding neighbor to open list: {neighbor}")

        return []

# Path to the roads image
roads_img_path = "roads.png"

# Creating maze object
maze = Maze(roads_img_path)

# Creating AStar object
astar = AStar(maze)

# Starting point and goal point
start_point = (109, 296)
goal_point = (190, 326)

# Ensure the start and goal points are not in obstacles
if maze.isPointInObstacles(start_point[0], start_point[1]):
    print("Start point is in an obstacle.")
if maze.isPointInObstacles(goal_point[0], goal_point[1]):
    print("Goal point is in an obstacle.")

# Find the path using A* algorithm
path = astar.astar(start_point, goal_point)

if not path:
    print("No path found.")
else:
    print(path)

