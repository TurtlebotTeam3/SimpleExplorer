import numpy as np
import matplotlib.pyplot as plt
import copy as cp


class Wavefront:

    def _set_start_and_goal(self, map, xStart, yStart):
        """
        """
        size = 1
        map[yStart - size : yStart + size + 1, xStart - size : xStart + size + 1 ] = 0
        #map[yGoal][xGoal] = 2
        map[yStart][xStart] = -2
        return map

    def _label_adjacent(self, map, xStart, yStart):
        """
        """
        size = len(map) - 50
        run = True
        while run == True:
            for y in range(50, size):
                for x in range(50, size):
                    if map[y][x] == 0:
                        highestAdjeacent = self._get_highest_adjeacent(map, x, y)
                        if highestAdjeacent[2] > 1:
                            map[y][x] = highestAdjeacent[2] + 1
            # check start
            highestAdjeacentStart = self._get_highest_adjeacent(map, xStart, yStart)
            if highestAdjeacentStart[2] > 1:
                run = False
        return map

    def _find_path(self, map, xStart, yStart):
        """
        """
        currentX = xStart
        currentY = yStart
        lastX = xStart
        lastY = yStart
        #currentValue = -2
        waypoints = []

        map[currentY][currentX] = -2

        run = True
        while run == True:
            nextLowestAdjeacent = self._get_next_lowest_adjeacent(map, currentX, currentY)

            print nextLowestAdjeacent[2]

            if nextLowestAdjeacent[2] != 2:
                lastX = currentX
                lastY = currentY
                currentX = nextLowestAdjeacent[0]
                currentY = nextLowestAdjeacent[1]

                map[nextLowestAdjeacent[1]][nextLowestAdjeacent[0]] = -555
            else:
                waypoints.append((currentX, currentY))
                map[nextLowestAdjeacent[1]][nextLowestAdjeacent[0]] = -666
                run = False
        return map , waypoints

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

    def _get_next_lowest_adjeacent(self, map, currentX, currentY):
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
            tempX = currentX
            tempY = currentY - 1
            tempValue = map[currentY - 1][currentX]
        # check bottom
        if (map[currentY + 1][currentX]) == tempValue - 1:
            tempX = currentX
            tempY = currentY + 1
            tempValue = map[currentY + 1][currentX]
        # check left
        if (map[currentY][currentX - 1]) == tempValue - 1:
            tempX = currentX - 1
            tempY = currentY
            tempValue = map[currentY][currentX - 1]
            # check right
        if (map[currentY][currentX + 1]) == tempValue - 1:
            tempX = currentX + 1
            tempY = currentY
            tempValue = map[currentY][currentX + 1]

        nextA = [tempX, tempY, tempValue]

        return nextA

    def run(self, map, xStart, yStart):
        """
        """
        print "Calculating Wavefront"
        map = cp.deepcopy(map)
        # Walls = 100 | Unknown = -1 | Free Space = 0
        # Walls need to be set to 1 to make algorithm work
        map[map == 100] = 1
        #Walls
        size = 50
        map[:,0:size] = 1
        map[:,len(map) - 1 - size : len(map)-1] = 1
        map[0:size] = 1
        map[len(map)- 1 - size : len(map)-1] = 1
        map[map == -1] = 2

        map = self._set_start_and_goal(map, xStart, yStart)
        np.savetxt("wayfrontGoals.csv", map , delimiter=",", fmt='%1.3f')

        print "Labeling Wavefront"
        map = self._label_adjacent(map, xStart, yStart)
        np.savetxt("wayfrontLabeling.csv", map , delimiter=",", fmt='%1.3f')       

        print "Path finding Wavefront"
        map, waypoints = self._find_path(map, xStart, yStart)
        np.savetxt("wayfrontPath.csv", map , delimiter=",", fmt='%1.3f')

        raw_input("ddddd")

        #print waypoints
        #raw_input("ddddd")
        return waypoints