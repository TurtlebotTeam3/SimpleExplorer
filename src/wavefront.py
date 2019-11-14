import numpy as np
import matplotlib.pyplot as plt
import copy as cp


class Wavefront:

    def _set_start_and_goal(self, map, xGoal, yGoal, xStart, yStart):
        """
        """
        map[yGoal][xGoal] = 2
        map[yStart][xStart] = -2
        return map

    def _label_adjacent(self, map, xStart, yStart):
        """
        """
        size = len(map)
        run = True
        while run == True:
            for y in range(0, size):
                for x in range(0, size):
                    if map[y][x] == 0:
                        highestAdjeacent = self._get_highest_adjeacent(map, x, y)
                        if highestAdjeacent[2] > 1:
                            map[y][x] = highestAdjeacent[2] + 1
            # check start
            highestAdjeacentStart = self._get_highest_adjeacent(map, xStart, yStart)
            if highestAdjeacentStart[2] > 1:
                run = False
        return map

    def _find_path(self, map, xStart, yStart, radius):
        """
        """
        currentX = xStart
        currentY = yStart
        lastX = xStart
        lastY = yStart
        direction = 'v'
        #currentValue = -2
        waypoints = []
        firstWayPoint = True

        run = True
        while run == True:
            nextLowestAdjeacent = self._get_next_lowest_adjeacent(
                map, currentX, currentY, radius)

            if nextLowestAdjeacent[2] != 2:
                lastX = currentX
                lastY = currentY
                currentX = nextLowestAdjeacent[0]
                currentY = nextLowestAdjeacent[1]

                #currentValue = currentValue - 1
                #map[currentY][currentX] = currentValue

                direction_changed, direction = self._detect_direction_change(direction, currentX, currentY, lastX, lastY)
                if direction_changed:
                    if firstWayPoint:
                        firstWayPoint = False
                    else:
                        waypoints.append((lastX, lastY))
                        # if len(waypoints) >= 2:
                        #    run = False

            elif (nextLowestAdjeacent[2]) == 2:
                waypoints.append((currentX, currentY))
                run = False
        return map , waypoints

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

    def _get_highest_adjeacent(self, map, currentX, currentY):
        """
        """
        tempX = 0
        tempY = 0
        tempValue = 0

        # check top
        if(map[currentY - 1][currentX] > tempValue):
            tempX = currentX
            tempY = currentY - 1
            tempValue = map[currentY - 1][currentX]
        # check right
        if (map[currentY][currentX + 1] > tempValue):
            tempX = currentX + 1
            tempY = currentY
            tempValue = map[currentY][currentX + 1]
        # check bottom
        if (map[currentY + 1][currentX] > tempValue):
            tempX = currentX
            tempY = currentY + 1
            tempValue = map[currentY + 1][currentX]
        # check left
        if (map[currentY][currentX - 1] > tempValue):
            tempX = currentX - 1
            tempY = currentY
            tempValue = map[currentY][currentX - 1]

        highestAdjeacent = [tempX, tempY, tempValue]

        return highestAdjeacent

    def _get_next_lowest_adjeacent(self, map, currentX, currentY, radius):
        """
        """
        tempX = 0
        tempY = 0
        tempValue = 0

        if map[currentY][currentX] == -2:
            highestAdjeacent = self._get_highest_adjeacent(
                map, currentX, currentY)
            tempValue = highestAdjeacent[2] + 1
        else:
            tempValue = map[currentY][currentX]

        # check top
        if (map[currentY - 1][currentX]) == tempValue - 1:
            row = currentY - 1
            col = currentX
            surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 100
            if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[currentY - 1][currentX] == -2:
                tempX = currentX
                tempY = currentY - 1
                tempValue = map[currentY - 1][currentX]
        # check bottom
        if (map[currentY + 1][currentX]) == tempValue - 1:
            row = currentY
            col = currentX + 1
            surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 100
            if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[currentY - 1][currentX] == -2:
                tempX = currentX
                tempY = currentY + 1
                tempValue = map[currentY + 1][currentX]
        # check left
        if (map[currentY][currentX - 1]) == tempValue - 1:
            row = currentY + 1
            col = currentX
            surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 100
            if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[currentY - 1][currentX] == -2:
                tempX = currentX - 1
                tempY = currentY
                tempValue = map[currentY][currentX - 1]
            # check right
        if (map[currentY][currentX + 1]) == tempValue - 1:
            row = currentY
            col = currentX - 1
            surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 100
            if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[currentY - 1][currentX] == -2:
                tempX = currentX + 1
                tempY = currentY
                tempValue = map[currentY][currentX + 1]

        nextA = [tempX, tempY, tempValue]

        return nextA

    def run(self, map, xGoal, yGoal, xStart, yStart, radius):
        """
        """
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = 1
        map = self._set_start_and_goal(
            map, xGoal, yGoal, xStart, yStart)

        map = self._label_adjacent(map, xStart, yStart)

        map, waypoints = self._find_path(map, xStart, yStart, radius)

        return waypoints

    def findUnknown(self, map, xStart, yStart, radius):
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = 1
        map[yStart][xStart] = -2
        # Set all unkown areas as goal
        map[map == -1] = 2
        map = self._label_adjacent(map, xStart, yStart)
        map, waypoints = self._find_path(map, xStart, yStart, radius)

        return waypoints


