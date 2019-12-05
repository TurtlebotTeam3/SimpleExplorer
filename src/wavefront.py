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
        allpoints = []
        first = True

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
                    if first:
                        first = False
                    waypoints.append((lastX, lastY))
                allpoints.append((currentX, currentY))

            elif (nextLowestAdjeacent[2]) == 2:
                waypoints.append((currentX, currentY))
                run = False
        return map , waypoints, allpoints

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

    def _get_next_lowest_adjeacent(self, map, currentX, currentY,radius):
        """
        """
        tempX = 0
        tempY = 0
        tempValue = 0
        found = False

        if map[currentY][currentX] == -2:
            highestAdjeacent = self._get_highest_adjeacent(
                map, currentX, currentY)
            tempValue = highestAdjeacent[2] + 1
        else:
            tempValue = map[currentY][currentX]

        # check top
        if not found and map[currentY - 1][currentX] == tempValue - 1:
            row = currentY - 1
            col = currentX
            #surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 1
            #if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[row][col] == -2:
            tempX = col
            tempY = row
            tempValue = map[row][col]
            found = True
       
        # check bottom
        if not found and map[currentY + 1][currentX] == tempValue - 1:
            row = currentY + 1
            col = currentX
            #surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 1
            #if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[row][col] == -2:
            tempX = col
            tempY = row
            tempValue = map[row][col]
            found = True
        
        # check left
        if not found and map[currentY][currentX - 1] == tempValue - 1:
            row = currentY
            col = currentX - 1
            #surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 1
            #if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[row][col] == -2:
            tempX = col
            tempY = row
            tempValue = map[row][col]
            found = True
        
        # check right
        if not found and map[currentY][currentX + 1] == tempValue - 1:
            row = currentY
            col = currentX + 1
            #surrounding_any_wall = map[row - radius : row + radius + 1, col - radius : col + radius + 1] == 1
            #if (not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1))) or map[row][col] == -2:
            tempX = col
            tempY = row
            tempValue = map[row][col]
            found = True

        nextA = [tempX, tempY, tempValue]

        return nextA

    def _move_waipoints_away_from_obstacles(self, map, waypoints, radius):
        waypoints = self._move_single_point(map, waypoints, radius)
        waypoints = self._move_point_pairs(map, waypoints, radius)
        return waypoints

    def _move_single_point(self, map, waypoints, radius):
        for i in range(len(waypoints) - 1, -1, -1):
            (x, y) = waypoints[i]
            x_org = cp.copy(x)
            y_org = cp.copy(y)
            all_clear = False
            max_iter = 8

            while not all_clear and max_iter != 0:
                max_iter -= 1
                all_clear = True
                # map[waypoints[i][1] - radius : waypoints[i][1] + radius + 1, waypoints[i][10] - radius : waypoints[i][0] + radius + 1] == 1
                any_wall_right = map[y - radius : y + radius + 1 , x + radius] == 1
                any_wall_left = map[y - radius : y + radius + 1, x - radius] == 1
                any_wall_top = map[y + radius, x - radius : x + radius + 1] == 1
                any_wall_bottom = map[y - radius, x - radius : x + radius + 1] == 1
                
                if np.any(any_wall_right): # and any(np.any(any_wall_right, axis = 1)):
                    x -= 1
                    all_clear = False
                elif np.any(any_wall_left): # and any(np.any(any_wall_left, axis = 1)):
                    x += 1
                    all_clear = False
                elif np.any(any_wall_top): # and any(np.any(any_wall_top, axis = 1)):
                    y -= 1
                    all_clear = False
                elif np.any(any_wall_bottom): # and any(np.any(any_wall_bottom, axis = 1)):
                    y += 1
                    all_clear = False

            if i > 0:
                if x_org != x:
                    waypoints[i] = (x, y)
                    (x1, y1) = waypoints[i - 1]
                    waypoints[i - 1] = (x, y1)
                
                if y_org != y:
                    waypoints[i] = (x, y)
                    (x1, y1) = waypoints[i - 1]
                    waypoints[i - 1] = (x1, y)

        return waypoints

    def _move_point_pairs(self, map, waypoints, radius):
        for i in range(len(waypoints) - 1, 0, -1):
            (x1, y1) = waypoints[i]
            (x2, y2) = waypoints[i - 1]
            all_clear = False
            max_iter = 8

            # check if y axis is clear
            if y1 == y2:
                while not all_clear and max_iter != 0:
                    max_iter -= 1
                    all_clear = True
                    any_wall_top = None
                    any_wall_bottom = None

                    # get a boolean array indicating if there is a wall above or below
                    if x1 < x2:
                        any_wall_top = map[y1 + radius, x1 : x2 + 1] == 1
                        any_wall_bottom = map[y1 - radius, x1: x2 + 1] == 1
                    else:
                        any_wall_top = map[y1 + radius, x2 : x1 + 1] == 1
                        any_wall_bottom = map[y1 - radius, x2: x1 + 1] == 1
                    
                    # when there is a wall then correct the y axis to clear them
                    if np.any(any_wall_top):
                        y1 = y2 = y1 - 1
                        all_clear = False
                    elif np.any(any_wall_bottom):
                        y1 = y2 = y1 + 1
                        all_clear = False

            # check if x axis is clear
            if x1 == x2:
                while not all_clear and max_iter != 0:
                    max_iter -= 1
                    all_clear = True
                    any_wall_right = None
                    any_wall_left = None
                    # get a boolean array indicating if there is a wall left or right
                    if y1 < y2:
                        any_wall_right = map[y1 : y2 + 1 , x1 + radius] == 1
                        any_wall_left = map[y1 : y2 + 1, x1 - radius] == 1
                    else:
                        any_wall_right = map[y2 : y1 + 1 , x1 + radius] == 1
                        any_wall_left = map[y2 : y1 + 1, x1 - radius] == 1

                    # when there is a wall then correct the x axis to clear them
                    if np.any(any_wall_right):
                        x1 = x2 = x1 - 1
                        all_clear = False
                    elif np.any(any_wall_left):
                        x1 = x2 = x1 + 1
                        all_clear = False
            
            waypoints[i] = (x1, y1)
            waypoints[i - 1] = (x2, y2)
        
        return waypoints


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

        map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)

        return waypoints, allpoints

    def findUnknown(self, map, xStart, yStart, radius):
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = 1
        map[yStart][xStart] = -2
        # Set all unkown areas as goal
        map[map == -1] = 2
        map = self._label_adjacent(map, xStart, yStart)
        map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)
        waypoints = self._move_waipoints_away_from_obstacles(map, waypoints, radius)
        return waypoints, allpoints


