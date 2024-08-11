#!/usr/bin/env python3

import heapq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point as ROSPoint
from std_msgs.msg import Bool, String
import cv2
import os

class Maze:
    def __init__(self):
        file_path = os.path.join(os.path.dirname(__file__), 'roads.png')
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Image file {file_path} does not exist")
        (thresh, self.roads) = cv2.threshold(cv2.imread(file_path, cv2.IMREAD_GRAYSCALE), 127, 255, cv2.THRESH_BINARY)

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
        if xCoord < 0 or xCoord >= self.roads.shape[1] or yCoord < 0 or yCoord >= self.roads.shape[0]:
            return True
        return self.roads[yCoord, xCoord] == 0

    def isValidPoint(self, parentNodeX, parentNodeY, childNodeX, childNodeY):
        line_points = self.bresenham_line(parentNodeX, parentNodeY, childNodeX, childNodeY)
        return not any(self.isPointInObstacles(x, y) for x, y in line_points)

    def visualize_path(self, path):
        display_img = cv2.imread(os.path.join(os.path.dirname(__file__), 'roads.png'))
        for i in range(len(path) - 1):
            cv2.line(display_img, path[i], path[i + 1], (255, 0, 0), 2) 
        cv2.imshow("Planned Path", display_img)
        cv2.waitKey(0)

    def point_transform(self, pnt):
        gazebo_x = (-1 * pnt[1] / 13.002) + 35.7567 + 0.01475
        gazebo_y = (-1 * pnt[0] / 13.002) + 22.6746 + 0.01475
        return (gazebo_x, gazebo_y)

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.publisher = self.create_publisher(ROSPoint, '/planned_path', 10)
        self.next_waypoint = self.create_subscription(Bool, '/next_waypoint', self.nextpoint_callback, 10)
        self.color_detected_sub = self.create_subscription(String, '/color_detected', self.color_detected_callback, 10)
        self.start_color_pub = self.create_publisher(Bool, '/start_color_detection', 10)
        self.stop_color_pub = self.create_publisher(Bool, '/stop_color_detection', 10)
        self.maze = Maze()
        self.start = (109, 296)
        self.current_position = self.start
        self.current_goal_index = 0
        self.goals = [(159, 208), (190, 326), (296, 142), (502, 539), (244, 640)]
        self.planned_path = []
        self.ready_next_point = True
        self.get_logger().info("Path Planner is initiallized")

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

        return []

    def plan_path(self, start, goal):
        return self.astar(start, goal)

    def robot_position_callback(self, msg):
        self.current_position = (msg.data[0], msg.data[1])

    def nextpoint_callback(self, msg):
        self.get_logger().info('Received /next_waypoint message')
        if msg.data:
            self.ready_next_point = True
            self.publish_next_point()

    def reached_goal(self, current_position, goal_position):
        return self.heuristic(current_position, goal_position) < 0.2

    def publish_next_point(self):
        if self.ready_next_point and self.planned_path:
            next_point = self.planned_path.pop(0)
            point_msg = ROSPoint()
            point_msg.x = float(next_point[0])
            point_msg.y = float(next_point[1])
            self.get_logger().info(f'Publishing next point: ({next_point[0]}, {next_point[1]})')
            self.publisher.publish(point_msg)
            self.ready_next_point = False

            if len(self.planned_path) == 10:
                self.get_logger().info(f"Near goal: {next_point}. Starting color detection.")
                self.start_color_pub.publish(Bool(data=True))
                self.detecting_color = True

        elif self.ready_next_point and not self.planned_path and self.current_goal_index < len(self.goals):
            self.get_logger().info(f'Planning path from {self.start} to {self.goals[self.current_goal_index]}')
            self.planned_path = self.plan_path(self.start, self.goals[self.current_goal_index])
            self.current_goal_index += 1
            if self.planned_path:
                self.start = self.planned_path[-1]
                self.publish_next_point()
            else:
                self.get_logger().warn('Path planning failed to find a path')

    def color_detected_callback(self, msg):
        self.get_logger().info(f"Color detected: {msg.data}. Stopping color detection.")
        self.stop_color_pub.publish(Bool(data=True))
        self.detecting_color = False
        self.publish_next_point()

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
