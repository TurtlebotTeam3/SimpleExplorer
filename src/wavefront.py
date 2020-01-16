import numpy as np
import matplotlib.pyplot as plt
import copy as cp


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

    def _label_adjacent(self, map, xStart, yStart):
        """
        """
        size = len(map)
        run = True
        while run == True:
            for y in range(0, size):
                for x in range(0, size):
                    if map[y][x] == 0:
                        highestAdjeacent = self._get_highest_adjeacent(
                            map, x, y)
                        if highestAdjeacent[2] >= self._value_goal:
                            map[y][x] = highestAdjeacent[2] + 1
            # check start
            highestAdjeacentStart = self._get_highest_adjeacent(
                map, xStart, yStart)
            if highestAdjeacentStart[2] >= self._value_goal:
                run = False
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
            # North: x,y-1
            # or (map[y - 1][x] > self._value_goal and (map[y][x] + 1) < map[y - 1][x]):
            if map[y - 1][x] == 0:
                map[y - 1][x] = map[y][x] + 1
                neighbours.append((x, y-1))
                if map[y][x] + 1 > max_val:
                    max_val = map[y][x] + 1

            # NortEast: x+1,y-1
            # or (map[y - 1][x + 1] > self._value_goal and (map[y][x] + 1) < map[y - 1][x + 1]):
            #if map[y - 1][x + 1] == 0:
            #   map[y - 1][x + 1] = map[y][x] + 1
            #   neighbours.append((x+1, y-1))
            #   if map[y][x] + 1 > max_val:
            #       max_val = map[y][x] + 1

            # East: x+1,y
            # or (map[y][x + 1] > self._value_goal and (map[y][x] + 1) < map[y][x + 1]):
            if map[y][x + 1] == 0:
                map[y][x + 1] = map[y][x] + 1
                neighbours.append((x+1, y))
                if map[y][x] + 1 > max_val:
                    max_val = map[y][x] + 1
            # SouthEast: x+1,y+1
            # or (map[y + 1][x + 1] > self._value_goal and (map[y][x] + 1) < map[y + 1][x + 1]):
            #if map[y + 1][x + 1] == 0:
            #   map[y + 1][x + 1] = map[y][x] + 1
            #   neighbours.append((x+1, y+1))
            #   if map[y][x] + 1 > max_val:
            #       max_val = map[y][x] + 1

            # South: x,y+1
            # or (map[y + 1][x] > self._value_goal and (map[y][x] + 1) < map[y + 1][x]):
            if map[y + 1][x] == 0:
               map[y + 1][x] = map[y][x] + 1
               neighbours.append((x, y+1))
               if map[y][x] + 1 > max_val:
                   max_val = map[y][x] + 1

            # SouthWest: x-1,y+1
            # or (map[y - 1][x - 1] > self._value_goal and (map[y][x] + 1) < map[y - 1][x - 1]):
            #if map[y + 1][x - 1] == 0:
            #   map[y + 1][x - 1] = map[y][x] + 1
            #   neighbours.append((x-1, y+1))
            #   if map[y][x] + 1 > max_val:
            #       max_val = map[y][x] + 1

            # West: x-1,y
            # or (map[y][x - 1] > self._value_goal and (map[y][x] + 1) < map[y][x - 1]):
            if map[y][x - 1] == 0:
                map[y][x - 1] = map[y][x] + 1
                neighbours.append((x-1, y))
                if map[y][x] + 1 > max_val:
                    max_val = map[y][x] + 1

            # NorthWest: x-1,y-1
            # or (map[y - 1][x - 1] > self._value_goal and (map[y][x] + 1) < map[y - 1][x - 1]):
            #if map[y - 1][x - 1] == 0:
            #    map[y - 1][x - 1] = map[y][x] + 1
            #    neighbours.append((x-1, y-1))
            #    if map[y][x] + 1 > max_val:
            #        max_val = map[y][x] + 1

        return map, max_val

    def _find_path(self, map, xStart, yStart, radius):
        """
        """
        currentX = xStart
        currentY = yStart
        lastX = xStart
        lastY = yStart
        direction = 'v'
        waypoints = []
        waypoints.append((xStart, yStart))
        allpoints = []
        first = True
        # 0 -> No / 1 -> Top / 2 -> Bottom / 3 -> Left / 4 -> Right / Top Left -> 5 / Top Right -> 6 / Bottom Right -> 7 / Bottom Left -> 8
        lastValueDirecton = 0

        run = True
        while run == True:
            nextLowestAdjeacent = self._get_next_lowest_adjeacent(
                map, currentX, currentY, radius, lastValueDirecton)
            lastValueDirecton = nextLowestAdjeacent[3]
            if nextLowestAdjeacent[0] == 0 and nextLowestAdjeacent[1] == 0 and nextLowestAdjeacent[2] == 0:
                return None, None, None

            if nextLowestAdjeacent[2] != self._value_goal:
                lastX = currentX
                lastY = currentY
                currentX = nextLowestAdjeacent[0]
                currentY = nextLowestAdjeacent[1]

                # currentValue = currentValue - 1
                # map[currentY][currentX] = currentValue

                direction_changed, direction = self._detect_direction_change(
                    direction, currentX, currentY, lastX, lastY)
                if direction_changed:
                    if first:
                        first = False
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
        #if(map[currentY - 1][currentX - 1] < tempValue) and map[currentY - 1][currentX - 1] != 1:
        #    tempX = currentX - 1
        #    tempY = currentY - 1
        #    tempValue = map[currentY - 1][currentX - 1]

        # check top
        if(map[currentY - 1][currentX] < tempValue) and map[currentY - 1][currentX] != 1:
            tempX = currentX
            tempY = currentY - 1
            tempValue = map[currentY - 1][currentX]

        # check top right
        #if(map[currentY - 1][currentX + 1] < tempValue) and map[currentY - 1][currentX + 1] != 1:
        #    tempX = currentX + 1
        #    tempY = currentY - 1
        #    tempValue = map[currentY - 1][currentX + 1]

        # check right
        if (map[currentY][currentX + 1] < tempValue) and map[currentY][currentX + 1] != 1:
            tempX = currentX + 1
            tempY = currentY
            tempValue = map[currentY][currentX + 1]

        # check bottom right
        #if (map[currentY + 1][currentX + 1] < tempValue) and map[currentY + 1][currentX + 1] != 1:
        #    tempX = currentX + 1
        #    tempY = currentY + 1
        #    tempValue = map[currentY + 1][currentX + 1]

        # check bottom
        if (map[currentY + 1][currentX] < tempValue) and map[currentY + 1][currentX] != 1:
            tempX = currentX
            tempY = currentY + 1
            tempValue = map[currentY + 1][currentX]

        # check bottom left
        #if (map[currentY + 1][currentX - 1] < tempValue) and map[currentY + 1][currentX - 1] != 1:
        #    tempX = currentX - 1
        #    tempY = currentY + 1
        #    tempValue = map[currentY + 1][currentX - 1]

        # check left
        if (map[currentY][currentX - 1] < tempValue) and map[currentY][currentX - 1] != 1:
            tempX = currentX - 1
            tempY = currentY
            tempValue = map[currentY][currentX - 1]

        # check bottom left
        #if (map[currentY - 1][currentX - 1] < tempValue) and map[currentY - 1][currentX - 1] != 1:
        #    tempX = currentX - 1
        #    tempY = currentY - 1
        #    tempValue = map[currentY - 1][currentX - 1]

        highestAdjeacent = [tempX, tempY, tempValue]

        return highestAdjeacent

    def _get_next_lowest_adjeacent(self, map, currentX, currentY, radius, lastValueDirecton):
        """
        """
        tempLastValueDirecton = lastValueDirecton

        tempTopLeftX = 0
        tempTopLeftY = 0
        tempTopLeftValue = 0

        tempTopX = 0
        tempTopY = 0
        tempTopValue = 0

        tempTopRightX = 0
        tempTopRightY = 0
        tempTopRightValue = 0

        tempRightX = 0
        tempRightY = 0
        tempRightValue = 0

        tempBottomRightX = 0
        tempBottomRightY = 0
        tempBottomRightValue = 0

        tempBottomX = 0
        tempBottomY = 0
        tempBottomValue = 0

        tempBottomLeftX = 0
        tempBottomLeftY = 0
        tempBottomLeftValue = 0

        tempLeftX = 0
        tempLeftY = 0
        tempLeftValue = 0

        if map[currentY][currentX] == -2:
            highestAdjeacent = self._get_first_adjeacent(
                map, currentX, currentY)
            tempValue = highestAdjeacent[2] + 1
        else:
            tempValue = map[currentY][currentX]

        # check top left
        # if map[currentY - 1][currentX - 1] == tempValue - 1:
        #    tempTopLeftX = currentX - 1
        #    tempTopLeftY = currentY - 1
        #    tempTopLeftValue = map[tempTopLeftY][tempTopLeftX]

        # check top
        if map[currentY - 1][currentX] == tempValue - 1:
            tempTopX = currentX
            tempTopY = currentY - 1
            tempTopValue = map[tempTopY][tempTopX]

        # check top right
        #if map[currentY - 1][currentX + 1] == tempValue - 1:
        #    tempTopRightX = currentX + 1
        #    tempTopRightY = currentY - 1
        #    tempTopRightValue = map[tempTopRightY][tempTopRightX]

        # check right
        if map[currentY][currentX + 1] == tempValue - 1:
            tempRightX = currentX + 1
            tempRightY = currentY
            tempRightValue = map[tempRightY][tempRightX]

        # check bottom right
        #if map[currentY + 1][currentX + 1] == tempValue - 1:
        #    tempBottomRightX = currentX + 1
        #    tempBottomRightY = currentY + 1
        #    tempBottomRightValue = map[tempBottomRightY][tempBottomRightX]

        # check bottom
        if map[currentY + 1][currentX] == tempValue - 1:
            tempBottomX = currentX
            tempBottomY = currentY + 1
            tempBottomValue = map[tempBottomY][tempBottomX]

        # check bottom left
        #if map[currentY + 1][currentX - 1] == tempValue - 1:
        #    tempBottomLeftX = currentX - 1
        #    tempBottomLeftY = currentY + 1
        #    tempBottomLeftValue = map[tempBottomLeftY][tempBottomLeftX]

        # check left
        if map[currentY][currentX - 1] == tempValue - 1:
            tempLeftX = currentX - 1
            tempLeftY = currentY
            tempLeftValue = map[tempLeftY][tempLeftX]

        # 0 -> No / 1 -> Top / 2 -> Bottom / 3 -> Left / 4 -> Right / Top Left -> 5 / Top Right -> 6 / Bottom Right -> 7 / Bottom Left -> 8

        # moved top
        if tempLastValueDirecton == 1 and tempTopValue != 0:
            return [tempTopX, tempTopY, tempTopValue, 1]

        # moved bottom
        elif tempLastValueDirecton == 2 and tempBottomValue != 0:
            return [tempBottomX, tempBottomY, tempBottomValue, 2]

        # moved left
        elif tempLastValueDirecton == 3 and tempLeftValue != 0:
            return [tempLeftX, tempLeftY, tempLeftValue, 3]

        # moved right
        elif tempLastValueDirecton == 2 and tempRightValue != 0:
            return [tempRightX, tempRightY, tempRightValue, 4]

        # moved top left
        #elif tempLastValueDirecton == 5 and tempTopLeftValue != 0:
        #    return [tempTopLeftX, tempTopLeftY, tempTopLeftValue, 5]

        # moved top right
        #elif tempLastValueDirecton == 6 and tempTopRightValue != 0:
        #    return [tempTopRightX, tempTopRightY, tempTopRightValue, 6]

        # moved bottom left
        #elif tempLastValueDirecton == 7 and tempBottomLeftValue != 0:
        #    return [tempBottomLeftX, tempBottomLeftY, tempBottomLeftValue, 7]

        # moved bottom right
        #elif tempLastValueDirecton == 8 and tempBottomRightValue != 0:
        #    return [tempBottomRightX, tempBottomRightY, tempBottomRightValue, 8]

        #elif tempTopLeftValue != 0:
        #    return [tempTopX, tempTopY, tempTopLeftValue, 5]

        elif tempTopValue != 0:
            return [tempTopX, tempTopY, tempTopValue, 1]

        #elif tempTopRightValue != 0:
        #    return [tempTopX, tempTopY, tempTopRightValue, 7]

        elif tempRightValue != 0:
            return [tempRightX, tempRightY, tempRightValue, 4]

        #elif tempBottomRightValue != 0:
        #    return [tempTopX, tempTopY, tempBottomRightValue, 8]

        elif tempBottomValue != 0:
            return [tempBottomX, tempBottomY, tempBottomValue, 2]

        #elif tempBottomLeftValue != 0:
        #    return [tempTopX, tempTopY, tempBottomLeftValue, 9]

        elif tempLeftValue != 0:
            return [tempLeftX, tempLeftY, tempLeftValue, 3]

        else:
            return [0, 0, 0, 0]

    def _move_waipoints_away_from_obstacles(self, map, waypoints, radius):
        for i in range(len(waypoints) - 1, -1, -1):
            (x1, y1) = waypoints[i]
            (x2, y2) = waypoints[i - 1]
            all_clear = False
            max_iter = 8

            # x, y = _move_single_point(self, map, x1, y1, radius)

            # check if y axis is clear
            if y1 == y2:

                while not all_clear and max_iter != 0:
                    max_iter -= 1
                    all_clear = True
                    any_wall_top = None
                    any_wall_bottom = None

                    # get a boolean array indicating if there is a wall above or below
                    if x1 < x2:
                        any_wall_top= map[y1 + radius, x1: x2 + 1] == 1
                        any_wall_bottom= map[y1 - radius, x1: x2 + 1] == 1
                    else:
                        any_wall_top= map[y1 + radius, x2: x1 + 1] == 1
                        any_wall_bottom= map[y1 - radius, x2: x1 + 1] == 1

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
                        any_wall_right = map[y1: y2 + 1, x1 + radius] == 1
                        any_wall_left = map[y1: y2 + 1, x1 - radius] == 1
                    else:
                        any_wall_right = map[y2: y1 + 1, x1 + radius] == 1
                        any_wall_left= map[y2: y1 + 1, x1 - radius] == 1

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

    def _move_single_point(self, map, x, y, radius):
        x_org = cp.copy(x)
        y_org = cp.copy(y)
        all_clear = False
        max_iter = 8

        while not all_clear and max_iter != 0:
            max_iter -= 1
            all_clear = True
            # map[waypoints[i][1] - radius : waypoints[i][1] + radius + 1, waypoints[i][10] - radius : waypoints[i][0] + radius + 1] == 1
            any_wall_right = map[y - radius: y + radius + 1, x + radius] == 1
            any_wall_left= map[y - radius: y + radius + 1, x - radius] == 1
            any_wall_top= map[y + radius, x - radius: x + radius + 1] == 1
            any_wall_bottom= map[y - radius, x - radius: x + radius + 1] == 1

            # and any(np.any(any_wall_right, axis = 1)):
            if np.any(any_wall_right):
                x -= 1
                all_clear = False
            # and any(np.any(any_wall_left, axis = 1)):
            elif np.any(any_wall_left):
                x += 1
                all_clear = False
            # and any(np.any(any_wall_top, axis = 1)):
            elif np.any(any_wall_top):
                y -= 1
                all_clear = False
            # and any(np.any(any_wall_bottom, axis = 1)):
            elif np.any(any_wall_bottom):
                y += 1
                all_clear = False

        # if i > 0:
            # x position changed
        #    if x_org != x:
        #        waypoints[i] = (x, y)
        #        (x1, y1) = waypoints[i - 1]
        #        waypoints[i - 1] = (x, y1)

        #    if y_org != y:
        #        waypoints[i] = (x, y)
        #        (x1, y1) = waypoints[i - 1]
        #        waypoints[i - 1] = (x1, y)

        return x, y

    def run(self, map, xGoal, yGoal, xStart, yStart, radius):
        """
        """
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = 1
        map = self._set_start_and_goal(
            map, xGoal, yGoal, xStart, yStart)

        # map = self._label_adjacent(map, xStart, yStart)
        map, max_val = self._label_cells(map, [(xGoal, yGoal)])

        map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)

        return waypoints, allpoints

    def find_unknown(self, map, xStart, yStart, radius):
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = self._value_wall
        map[yStart][xStart] = self._value_start

        # map = self._label_adjacent(map, xStart, yStart)
        np.savetxt("map.csv", map, delimiter=",", fmt='%1.3f')
        # map, goals = self._find_all_unknown(map, radius)
        map, goals = self._find_all_unknown(map, 0)
        np.savetxt("map_goals.csv", map, delimiter=",", fmt='%1.3f')
        map, max_val = self._label_cells(map, goals)
        # map, max_val = self._label_cells(map, [(xStart, yStart)])
        np.savetxt("map_labeled.csv", map, delimiter=",", fmt='%1.3f')

        map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)

        if waypoints == None and allpoints == None:
            return None, None
        else:
            waypoints = self._move_waipoints_away_from_obstacles(map, waypoints, 3)
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
