import numpy as np
import matplotlib.pyplot as plt
import copy as cp
from directionEnum import Direction

class Wavefront:

    def __init__(self):
        self._value_goal = 2
        self._value_start = -2
        self._value_wall = 1

    def _set_start_and_goal(self, map, xGoal, yGoal, xStart, yStart):
        """
        """
        map[yGoal][xGoal] = self._value_goal
        map[yStart][xStart] = self._value_start
        return map

    def _label_cells(self, map, neighbours):
        """
        Labels the map.

        Parameters:
        map: map to label
        start ([]): Array of Touple (x,y) of starting position

        Returns:
        map: the labeled map
        max_val: max value during labeling
        """

        max_val = 0
        while len(neighbours) != 0:
            (x, y) = neighbours.pop(0)
   
            if map[y - 1][x] == 0:
                map[y - 1][x] = map[y][x] + 1
                neighbours.append((x, y-1))
                if map[y][x] + 1 > max_val:
                    max_val = map[y][x] + 1

            if map[y][x + 1] == 0:
                map[y][x + 1] = map[y][x] + 1
                neighbours.append((x+1, y))
                if map[y][x] + 1 > max_val:
                    max_val = map[y][x] + 1

            if map[y + 1][x] == 0:
               map[y + 1][x] = map[y][x] + 1
               neighbours.append((x, y+1))
               if map[y][x] + 1 > max_val:
                   max_val = map[y][x] + 1

            if map[y][x - 1] == 0:
                map[y][x - 1] = map[y][x] + 1
                neighbours.append((x-1, y))
                if map[y][x] + 1 > max_val:
                    max_val = map[y][x] + 1

        return map, max_val

    def _find_path(self, map, xStart, yStart, radius):
        """
        """
        currentX = xStart
        currentY = yStart
        lastX = xStart
        lastY = yStart
        waypoints = []
        allpoints = []
        # 0 -> No / 1 -> Top / 2 -> Bottom / 3 -> Left / 4 -> Right / Top Left -> 5 / Top Right -> 6 / Bottom Right -> 7 / Bottom Left -> 8
        lastDirection = Direction.Neutral

        run = True
        while run == True:
            nextLowestAdjeacent = self._get_next_lowest_adjeacent(
                map, currentX, currentY)
            if nextLowestAdjeacent[0] == 0 and nextLowestAdjeacent[1] == 0 and nextLowestAdjeacent[2] == 0:
                return None, None, None

            if nextLowestAdjeacent[2] != self._value_goal:
                lastX = currentX
                lastY = currentY
                currentX = nextLowestAdjeacent[0]
                currentY = nextLowestAdjeacent[1]
                direction =nextLowestAdjeacent[3]
                # currentValue = currentValue - 1
                # map[currentY][currentX] = currentValue

                #direction_changed, direction = self._detect_direction_change(direction, currentX, currentY, lastX, lastY)
                if direction != lastDirection:
                    lastDirection = direction
                    waypoints.append((lastX, lastY))
                allpoints.append((currentX, currentY))

            elif (nextLowestAdjeacent[2]) == self._value_goal:
                waypoints.append((currentX, currentY))
                run = False
        return map, waypoints, allpoints

    def _detect_direction_change(self, direction, current_x, current_y, last_x, last_y):
        """
        Detects when movemend changes from vertical to horizontal or horizontal to vertical
        """
        direction_changed = False
        if current_x != last_x and direction == 'v':
            direction = 'h'
            direction_changed = True
        if current_y != last_y and direction == 'h':
            direction = 'v'
            direction_changed = True

        return direction_changed, direction

    def _get_first_adjeacent(self, map, currentX, currentY):
        """
        """
        tempX = 0
        tempY = 0
        tempValue = 147456

        # check left top
        if(map[currentY - 1][currentX - 1] < tempValue) and map[currentY - 1][currentX - 1] != 1:
            tempX = currentX - 1
            tempY = currentY - 1
            tempValue = map[currentY - 1][currentX - 1]

        # check top
        if(map[currentY - 1][currentX] < tempValue) and map[currentY - 1][currentX] != 1:
            tempX = currentX
            tempY = currentY - 1
            tempValue = map[currentY - 1][currentX]

        # check top right
        if(map[currentY - 1][currentX + 1] < tempValue) and map[currentY - 1][currentX + 1] != 1:
            tempX = currentX + 1
            tempY = currentY - 1
            tempValue = map[currentY - 1][currentX + 1]

        # check right
        if (map[currentY][currentX + 1] < tempValue) and map[currentY][currentX + 1] != 1:
            tempX = currentX + 1
            tempY = currentY
            tempValue = map[currentY][currentX + 1]

        # check bottom right
        if (map[currentY + 1][currentX + 1] < tempValue) and map[currentY + 1][currentX + 1] != 1:
            tempX = currentX + 1
            tempY = currentY + 1
            tempValue = map[currentY + 1][currentX + 1]

        # check bottom
        if (map[currentY + 1][currentX] < tempValue) and map[currentY + 1][currentX] != 1:
            tempX = currentX
            tempY = currentY + 1
            tempValue = map[currentY + 1][currentX]

        # check bottom left
        if (map[currentY + 1][currentX - 1] < tempValue) and map[currentY + 1][currentX - 1] != 1:
            tempX = currentX - 1
            tempY = currentY + 1
            tempValue = map[currentY + 1][currentX - 1]

        # check left
        if (map[currentY][currentX - 1] < tempValue) and map[currentY][currentX - 1] != 1:
            tempX = currentX - 1
            tempY = currentY
            tempValue = map[currentY][currentX - 1]

        # check bottom left
        if (map[currentY - 1][currentX - 1] < tempValue) and map[currentY - 1][currentX - 1] != 1:
            tempX = currentX - 1
            tempY = currentY - 1
            tempValue = map[currentY - 1][currentX - 1]

        highestAdjeacent = [tempX, tempY, tempValue]

        return highestAdjeacent

    def _get_next_lowest_adjeacent(self, map, currentX, currentY):
        """
        """
        currentValue = 0

        nextX = 0
        nextY = 0
        nextValue = 0
        # 0 -> No
        # 1 -> Top
        # 2 -> Bottom
        # 3 -> Left
        # 4 -> Right
        # 5 -> Top Left
        # 6 -> Top Right
        # 7 -> Bottom Right
        # 8 -> Bottom Left
        direction = Direction.Neutral

        if map[currentY][currentX] == -2:
            highestAdjeacent = self._get_first_adjeacent(
                map, currentX, currentY)
            return [highestAdjeacent[0], highestAdjeacent[1], highestAdjeacent[2], Direction.Neutral]
        else:
            nextValue = currentValue = map[currentY][currentX]

        # check top left
        if map[currentY - 1][currentX - 1] == currentValue - 2 and map[currentY - 1][currentX - 1] >=  self._value_goal:
            nextX = currentX - 1
            nextY = currentY - 1
            nextValue = map[nextY][nextX]
            direction = Direction.NorthWest

        # check top
        if map[currentY - 1][currentX] == currentValue - 1 and map[currentY - 1][currentX] >=  self._value_goal and map[currentY - 1][currentX] < nextValue:
            nextX = currentX
            nextY = currentY - 1
            nextValue = map[nextY][nextX]
            direction = Direction.North

        # check top right
        if map[currentY - 1][currentX + 1] == currentValue - 2 and map[currentY - 1][currentX + 1] >=  self._value_goal and map[currentY - 1][currentX + 1] < nextValue:
            nextX = currentX + 1
            nextY = currentY - 1
            nextValue = map[nextY][nextX]
            direction = Direction.NorthEast

        # check right
        if map[currentY][currentX + 1] == currentValue - 1 and map[currentY][currentX + 1] >=  self._value_goal and map[currentY][currentX + 1] < nextValue:
            nextX = currentX + 1
            nextY = currentY
            nextValue = map[nextY][nextX]
            direction = Direction.East

        # check bottom right
        if map[currentY + 1][currentX + 1] == currentValue - 2 and map[currentY + 1][currentX + 1] >=  self._value_goal and map[currentY + 1][currentX + 1] < nextValue:
            nextX = currentX + 1
            nextY = currentY + 1
            nextValue = map[nextY][nextX]
            direction = Direction.SouthEast

        # check bottom
        if map[currentY + 1][currentX] == currentValue - 1 and map[currentY + 1][currentX] >=  self._value_goal and map[currentY + 1][currentX] < nextValue:
            nextX = currentX
            nextY = currentY + 1
            nextValue = map[nextY][nextX]
            direction = Direction.South

        # check bottom left
        if map[currentY + 1][currentX - 1] == currentValue - 2 and map[currentY + 1][currentX - 1] >=  self._value_goal and map[currentY + 1][currentX - 1] < nextValue:
            nextX = currentX - 1
            nextY = currentY + 1
            nextValue = map[nextY][nextX]
            direction = Direction.SouthWest

        # check left
        if map[currentY][currentX - 1] == currentValue - 1 and map[currentY][currentX - 1] >=  self._value_goal and map[currentY][currentX - 1] < nextValue:
            nextX = currentX - 1
            nextY = currentY
            nextValue = map[nextY][nextX]
            direction = Direction.West

        return [nextX, nextY, nextValue, direction]

    def find_path_to_goal(self, map, xGoal, yGoal, xStart, yStart, radius):
        """
        """
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = 1
        map = self._set_start_and_goal( map, xGoal, yGoal, xStart, yStart)

        map, _ = self._label_cells(map, [(xGoal, yGoal)])

        map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)

        return waypoints, allpoints

    def find_unknown(self, map, xStart, yStart, radius):
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = self._value_wall
        map[yStart][xStart] = self._value_start

        np.savetxt("map.csv", map, delimiter=",", fmt='%1.3f')
        map, goals = self._find_all_unknown(map, 0)
        np.savetxt("map_goals.csv", map, delimiter=",", fmt='%1.3f')
        map, _ = self._label_cells(map, goals)
        np.savetxt("map_labeled.csv", map, delimiter=",", fmt='%1.3f')

        map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)

        if waypoints == None and allpoints == None:
            return None, None
        else:
            return waypoints, allpoints


    def _find_all_unknown(self, map, robot_radius):
        """
        Find coordinates of all spots that are unkown:

        Parameters:
        map: the unlabled map

        Returns:
        map: the updated map with the goals set
        unknown_spots ([]): Array of tuple (x,y) of the goals
        """
        num_rows = len(map)
        num_cols = len(map[0])

        list_unknown_spots = []

        for row in range(robot_radius, num_rows - robot_radius):
            for col in range(robot_radius, num_cols - robot_radius):
                if map[row][col] == 0:
                    addGoal = False
                    surrounding_any_wall = map[row - robot_radius: row + robot_radius + 1, col - robot_radius: col + robot_radius + 1] == self._value_wall
                    if map[row - 1][col] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                        map[row][col] = self._value_goal
                        addGoal = True
                    if map[row + 1][col] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                        map[row][col] = self._value_goal
                        addGoal = True
                    if map[row][col - 1] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                        map[row][col] = self._value_goal
                        addGoal = True
                    if map[row][col + 1] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                        map[row][col] = self._value_goal
                        addGoal = True

                    if addGoal:
                        list_unknown_spots.append((col, row))

        return map, list_unknown_spots
