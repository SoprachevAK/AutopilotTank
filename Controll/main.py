import math
import time
import arcade
import numpy as np
from pygorithm.geometry.vector2 import Vector2
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

AREA_SIZE = Vector2(3, 3)
SIZE_OF_CELL = Vector2(0.2, 0.2)
THRESHOLD = 0.1
ACCESS_RAY_COUNT = 10
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 600
SCREEN_TITLE = "Grid"
PADDING = 10

IDLE_STATE = "IDLE_STATE"
SCAN_STATE = "SCAN_STATE"
MOVE_STATE = "MOVE_STATE"
FIND_PATH_STATE = "FIND_PATH_STATE"
END_TASK_STATE = "END_TASK_STATE"
ENABLE_ARCADE = True
MAX_LENGHT_SENSOR = 0

need_rerender = True
def createWindow():
    arcade.open_window(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    arcade.set_background_color(arcade.color.WHITE)
    arcade.schedule(main, 1 / 30)
    arcade.run()


def fakeArcade():
    while True:
        main(1 / 30)
        time.sleep(1 / 30)


def render():
    global path_to_ride
    if not ENABLE_ARCADE:
        return
    arcade.start_render()
    max_ = max(grid_size.x, grid_size.y)
    width = (SCREEN_WIDTH - PADDING * 2) // max_
    heigth = (SCREEN_HEIGHT - PADDING * 2) // max_
    for i in range(0, len(grid)):
        for j in range(0, len(grid[i])):
            color = arcade.color.GRAY
            if grid[i][j] > ACCESS_RAY_COUNT:
                color = arcade.color.GREEN
            elif grid[i][j] < -ACCESS_RAY_COUNT:
                color = arcade.color.RED_DEVIL
            arcade.draw_rectangle_filled(width // 2 + PADDING + i * width, heigth // 2 + PADDING + j * heigth, width - 3, heigth - 3, color=color)

    pos = translateScreenPosition(tank_position)
    arcade.draw_rectangle_filled(pos.x, pos.y, 20, 40, color=arcade.color.BLUE, tilt_angle=-tank_yaw)

    head_pos = translateScreenPosition(Vector2(0, 0.03).rotate(math.radians(tank_yaw)) + tank_position)
    line_end = translateScreenPosition(Vector2(0, head_dist).rotate(math.radians(tank_yaw + head_yaw)) + tank_position)
    arcade.draw_line(head_pos.x, head_pos.y, line_end.x, line_end.y, color=arcade.color.PURPLE, line_width=5)
    arcade.draw_rectangle_filled(head_pos.x, head_pos.y, 15, 15, color=arcade.color.ORANGE, tilt_angle=-(head_yaw + tank_yaw))

    if len(path_to_ride) > 0:
        t = translateScreenPosition(path_to_ride[0])
        arcade.draw_line(pos.x, pos.y, t.x, t.y, color=arcade.color.OCEAN_BOAT_BLUE, line_width=2)
        for i in range(1, len(path_to_ride)):
            f = translateScreenPosition(path_to_ride[i - 1])
            t = translateScreenPosition(path_to_ride[i])
            arcade.draw_line(f.x, f.y, t.x, t.y, color=arcade.color.OCEAN_BOAT_BLUE, line_width=2)


def translateScreenPosition(tank_pos):
    w = SCREEN_WIDTH - 2 * PADDING
    h = SCREEN_HEIGHT - 2 * PADDING
    return Vector2(tank_pos.x / (SIZE_OF_CELL.x * grid_size.x) * w + SCREEN_WIDTH / 2,
                   tank_pos.y / (SIZE_OF_CELL.y * grid_size.y) * h + SCREEN_HEIGHT / 2)


def findPath(from_v, to_v):
    x_f = int(from_v.x)
    y_f = int(from_v.y)
    x_t = int(to_v.x)
    y_t = int(to_v.y)
    oldPathFrom = pathMatrix[y_f][x_f]
    oldPathTo = pathMatrix[y_t][x_t]
    pathMatrix[y_f][x_f] = 1
    pathMatrix[y_t][x_t] = 1
    gridPath = Grid(matrix=pathMatrix)
    start_p = gridPath.node(x_f, y_f)
    end = gridPath.node(x_t, y_t)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.if_at_most_one_obstacle)
    path, runs = finder.find_path(start_p, end, gridPath)
    print(gridPath.grid_str(start=start_p, end=end, path=path))
    pathMatrix[y_f][x_f] = oldPathFrom
    pathMatrix[y_t][x_t] = oldPathTo
    return path


ROTATE = "ROTATE"
MOVING = "MOVING"
moving_type = MOVING
moving_dir = 0
def tank_move(move_type, dir):
    global moving_dir, moving_type
    moving_type = move_type
    moving_dir = dir


def tank_move_tick():
    global tank_yaw, tank_position
    if moving_type == MOVING:
        tank_position += moving_dir * Vector2(0, 0.03).rotate(math.radians(tank_yaw)) * 0.3
    else:
        tank_yaw += np.sign(moving_dir) * 4


def angle_betwen_vectors(v1, v2):
    vec1 = [v1.x, v1.y]
    vec2 = [v2.x, v2.y]
    unit_vector_1 = vec1 / np.linalg.norm(vec1)
    unit_vector_2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = math.degrees(np.arccos(dot_product))
    vec31 = np.array([vec1[0], vec1[1], 0])
    vec32 = np.array([vec2[0], vec2[1], 0])
    signum = np.sign(np.dot([0, 0, 1], np.cross(vec31, vec32)))
    if angle == 180:
        return angle
    return angle * signum


is_rotation = False
path_to_ride = []
def rideToTarget():
    global tank_position, tank_target_position, is_rotation
    if len(path_to_ride) == 0:
        tank_move(MOVING, 0)
        change_state(SCAN_STATE)
        return
    if (tank_position - path_to_ride[0]).magnitude() < 0.05:
        path_to_ride.remove(path_to_ride[0])
        if len(path_to_ride) > 0:
            is_rotation = True
        return
    angle = angle_betwen_vectors(Vector2(0, 1).rotate(math.radians(tank_yaw)), path_to_ride[0] - tank_position)
    if is_rotation:
        if abs(angle) > 3:
            tank_move(ROTATE, angle)
        else:
            tank_move(MOVING, 0)
            is_rotation = False
    else:
        if abs(angle) < 5:
            tank_move(MOVING, 1)
        else:
            is_rotation = True
            tank_move(MOVING, 0)


head_dir = -1
def serial_read():
    global head_yaw, head_dist, head_dir
    if head_yaw < -90 or head_yaw > 90:
        head_dir *= -1
    head_dist = abs(math.sin(math.radians(head_yaw + 30)) * 0.5) + 0.1
    head_yaw += head_dir * 5


count_change = 0
old_yaw = 1
def scan():
    global count_change, old_yaw, head_yaw
    if (old_yaw > 0 and head_yaw < 0) or (old_yaw < 0 and head_yaw > 0):
        count_change += 1
        old_yaw = head_yaw
    if count_change > 3:
        change_state(FIND_PATH_STATE)

    vec = Vector2(0, head_dist).rotate(math.radians(tank_yaw + head_yaw))
    vec_dist = vec.magnitude_squared()
    little_dist = 0
    step = SIZE_OF_CELL.x / 3
    vec = vec.normalize() * step
    ind = 0
    while little_dist < vec_dist:
        pos__ = vec * ind
        little_dist = pos__.magnitude_squared()
        cell__ = translateToCell(pos__ + tank_position)
        if 0 <= cell__.x < grid_size.x and 0 <= cell__.y < grid_size.y:
            updateGrid(cell__, grid[cell__.x][cell__.y] + 1)
        ind += 1
    cell__ = translateToCell(vec * ind + tank_position)
    if 0 <= cell__.x < grid_size.x and 0 <= cell__.y < grid_size.y and (vec * ind).magnitude() < MAX_LENGHT_SENSOR:
        updateGrid(cell__, grid[cell__.x][cell__.y] - 5)


def findPathToNearest():
    global path_to_ride
    pos = findNearestCell()
    print(pos)
    if pos:
        path_to_ride = findPath(translateToCell(tank_position), pos)
        path_to_ride = [Vector2(x[0], x[1]) for x in path_to_ride]
        if len(path_to_ride) > 2:
            change_dir = path_to_ride[1] - path_to_ride[0]
            new_path = []
            for i in range(1, len(path_to_ride)):
                ite = path_to_ride[i] - path_to_ride[i - 1]
                if (change_dir.x != ite.x) or (change_dir.y != ite.y):
                    new_path.append(path_to_ride[i])
                    change_dir = path_to_ride[i] - path_to_ride[i - 1]
            if path_to_ride[len(path_to_ride) - 1] not in new_path:
                new_path.append(path_to_ride[len(path_to_ride) - 1])
            path_to_ride = new_path
        path_to_ride = [translateToTankPos(x) for x in path_to_ride]
        if len(path_to_ride) > 0:
            change_state(MOVE_STATE)
        else:
            change_state(END_TASK_STATE)
    else:
        change_state(END_TASK_STATE)


def calculateDistance():
    tempYaw = tank_yaw % 90
    if tempYaw == 0:
        return 1
    return math.sin(math.radians(90)) / math.sin(math.radians(90 - tempYaw))


def findNearestCell():
    cell = translateToTankPos(translateToCell(tank_position))
    forward = Vector2(0, 1).rotate(math.radians(tank_yaw)).normalize() * SIZE_OF_CELL.y * calculateDistance()
    back = forward.rotate(math.radians(180)) + cell
    back = translateToCell(back)
    cell = translateToCell(tank_position)
    minDis = 100
    minCell = None
    for step in range(1, 10):
        if minCell != None:
            break
        xy = cell + Vector2(step, step)
        for dir in [Vector2(0, -1), Vector2(-1, 0), Vector2(0, 1), Vector2(1, 0)]:
            while abs(xy.y - (cell.y + step * dir.y)) > 0 if dir.y != 0 else (
                    abs(xy.x - (cell.x + step * dir.x)) > 0 if dir.x != 0 else False):
                if (xy.x < grid_size.x and xy.y < grid_size.y) and (xy.x >= 0 and xy.y >= 0):
                    if -ACCESS_RAY_COUNT < grid[xy.x][xy.y] < ACCESS_RAY_COUNT:
                        path = findPath(cell, xy)
                        if len(path) != 0:
                            newMin = (back - xy).magnitude_squared()
                            if minDis > newMin or minCell == None:
                                print("MinDist = ",end=' ')
                                print(newMin)
                                minDis = newMin
                                minCell = xy
                xy += dir
    return minCell


def translateToCell(pos):
    return Vector2(math.floor(pos.x / SIZE_OF_CELL.x + grid_size.x / 2),
                   math.floor(pos.y / SIZE_OF_CELL.y + grid_size.y / 2))


def translateToTankPos(pos):
    return Vector2(SIZE_OF_CELL.x * (pos.x - (grid_size.x / 2)) + SIZE_OF_CELL.x / 2,
                   SIZE_OF_CELL.y * (pos.y - (grid_size.y / 2)) + SIZE_OF_CELL.y / 2)


def updateGrid(cell_pos, val):
    global grid, pathMatrix
    grid[cell_pos.x][cell_pos.y] = val
    pathMatrix[cell_pos.y][cell_pos.x] = 1 if val > ACCESS_RAY_COUNT else 0


def readSenser():
    for i in range(0, 1000000):
        print("Read Senser")


target_cell = Vector2(0, 0)
current_cell = Vector2(0, 0)
grid_size = Vector2(0, 0)
grid = []
pathMatrix = []
tank_yaw = 0
tank_position = Vector2(0.0, 0.0)
tank_position_delta = Vector2(0.0, 0.0)
tank_target_position = Vector2(0, 0)
head_yaw = 0
head_dist = 0.3
state = SCAN_STATE


def change_state(st):
    global state, count_change, old_yaw, path_to_ride, is_rotation
    print("From  " + state + " to " + st)
    if st == SCAN_STATE:
        old_yaw = head_yaw
        count_change = 0
    elif st == MOVE_STATE:
        is_rotation = True
    state = st


def main(dely_time):
    render()
    serial_read()
    tank_move_tick()
    if state == IDLE_STATE:
         pass
    elif state == MOVE_STATE:
        rideToTarget()
    elif state == SCAN_STATE:
        scan()
    elif state == FIND_PATH_STATE:
        findPathToNearest()
    elif state == END_TASK_STATE:
        #print("End Task")
        return
    else:
        print("Error")


def start():
    global SIZE_OF_CELL, target_cell, current_cell, grid_size, grid, pathMatrix
    grid_size = Vector2(int(AREA_SIZE.x / SIZE_OF_CELL.x), int(AREA_SIZE.y / SIZE_OF_CELL.y))
    if grid_size.x % 2 == 0: grid_size.x += 1
    if grid_size.y % 2 == 0: grid_size.y += 1
    SIZE_OF_CELL = Vector2(AREA_SIZE.x / grid_size.x, AREA_SIZE.y / grid_size.y)

    target_cell = translateToCell(Vector2(0, 0))
    current_cell = translateToCell(Vector2(0, 0))
    for x in range(0, grid_size.x):
        line = []
        for y in range(0, grid_size.y):
            line.append(0)
        grid.append(line)
    for x in range(0, grid_size.x):
        line = []
        for y in range(0, grid_size.y):
            line.append(0)
        pathMatrix.append(line)

    # array = [
    #     [1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1]
    # ]
    # array = [
    #     [0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0]
    # ]
    for i in range(3, 12):
        updateGrid(Vector2(i, 11), -1000)
        updateGrid(Vector2(i, 3), -1000)
    for i in range(3, 11):
        updateGrid(Vector2(11, i), -1000)
        updateGrid(Vector2(3, i), -1000)
    # for i in range(0, 5):
    #     for j in range(0, 5):
    #         if array[i][j] == 1:
    #             updateGrid(Vector2(i, j), 200)
    if ENABLE_ARCADE:
        createWindow()
    else:
        fakeArcade()


start()




