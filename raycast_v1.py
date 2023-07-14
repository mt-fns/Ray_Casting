import math
import pygame
import numpy as np
import random

degree = 10 # degree between rays
ray_center = [500, 200]
radius = 20
moves = ["UP", "DOWN", "LEFT", "RIGHT"]

def random_move():
    rand_ind = random.randint(0, 3)
    return moves[rand_ind]


def rotate_vector(x, y, rad):
    return (x * math.cos(rad) - y * math.sin(rad), x * math.sin(rad) + y * math.cos(rad))

def create_rays(initial_ray):
    deg = 0
    rays = []

    rays.append(initial_ray)
    while deg < 360:
        deg += degree
        rad = math.radians(deg)
        new_pos = rotate_vector(initial_ray.dir[0], initial_ray.dir[1],
                                rad)  # rotating a vector by x degrees from the initial ray's position
        new_ray = Ray(ray_center, new_pos)
        rays.append(new_ray)
    rays.pop()

    return rays

def create_walls(n):
    """Creates n random walls"""
    walls = []
    for i in range(n):
        x1 = random.randint(0, 900)
        y1 = random.randint(0, 600)
        y2 = random.randint(0, 600)
        x2 = random.randint(0, 900)
        wall = Wall((x1, y1), (x2, y2), str(i))
        walls.append(wall)
    return walls


class Wall():
    def __init__(self, start, end, name):
        self.start = start  # tuple
        self.end = end  # tuple -> end point of the wall
        self.reflect = False  # reflect rays if true
        self.name = name

class Ray():
    def __init__(self, pos, direction):
        self.pos = pos  # tuple
        self.dir = direction  # tuple -> add to the pos tuple to get the end point of the ray
        self.end = (pos[0] + direction[0], pos[1] + direction[1])
        self.reflect_origin = None  # boundary the ray was reflected from

    def hit_boundary(self, wall):
        x1 = wall.start[0]
        y1 = wall.start[1]
        x2 = wall.end[0]
        y2 = wall.end[1]

        x3 = self.pos[0]
        y3 = self.pos[1]
        x4 = self.end[0]
        y4 = self.end[1]

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        if denom == 0:  # ray and wall are parallel
            return False

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom  # line-line intersection
        u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom
        if 0 < t < 1:
            if u > 0:
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                return (x, y)

        else:
            return False

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def dotproduct(self, v1, v2):
        return sum((a * b) for a, b in zip(v1, v2))

    def length(self, v):
        return math.sqrt(self.dotproduct(v, v))

    def find_angle(self, bound, intersection):
        ray_vector = np.array([intersection[0] - self.pos[0], intersection[1] - self.pos[1]])  # subtract initial coords with end coords to get the vector
        bound_vector = np.array([bound.end[0] - bound.start[0], bound.end[1] - bound.start[1]])

        return math.acos(self.dotproduct(ray_vector, bound_vector) / (self.length(ray_vector) * self.length(bound_vector)))

    def project_vector(self, n, v):
        """ project ray onto normal vector """
        n = np.array(n)
        v = np.array(v)
        return n * np.dot(v, n) / np.dot(n, n)

    def normal_vector(self, bound, intersection):
        """ find the normal vector by rotating 90 degrees """
        bound_vector = [bound.end[0] - bound.start[0], bound.end[1] - bound.start[1]]
        normal_rad = np.deg2rad(-90)
        rotate_bound = rotate_vector(bound_vector[0], bound_vector[1], normal_rad)
        normal_vector = (intersection[0] + rotate_bound[0], intersection[1] + rotate_bound[1])
        return normal_vector

    def reflect_vector(self, n, v):
        projected = self.project_vector(n, v)
        vector_difference = (projected[0] - v[0], projected[1] - v[1])
        # print("project", projected)
        # print("reflect", vector_difference)
        return (projected[0] + vector_difference[0], projected[1] + vector_difference[1])

    def rotate_vector(self, x, y, rad):
        return (x * math.cos(rad) - y * math.sin(rad), x * math.sin(rad) + y * math.cos(rad))




# bound1 = Wall((300, 100), (200, 300), "1")
# bound2 = Wall((700, 100), (800, 300), "2")
# bound3 = Wall((300, 500), (700, 600), "3")
# bound3.reflect = True
# bound2.reflect = True
# bound_list.append(bound2)
# bound_list.append(bound1)
#bound_list.append(bound3)

bound_list = create_walls(10)

pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dragging = False
offset_x = 0
offset_y = 0

while running:


    move = random_move()
    ray1 = Ray(ray_center, (100, 0))  # initial ray to be used in rendering the rest
    ray_list = create_rays(ray1)
    surface = pygame.Surface((screen.get_width(), screen.get_height()), pygame.SRCALPHA)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                mouse_x, mouse_y = event.pos
                distance = math.sqrt((mouse_x - ray_center[0]) ** 2 + (mouse_y - ray_center[1]) ** 2)
                if distance < radius:
                    dragging = True

                    offset_x = ray_center[0] - mouse_x
                    offset_y = ray_center[1] - mouse_y

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if dragging:
                mouse_x, mouse_y = event.pos
                ray_center[0] = mouse_x + offset_x
                ray_center[1] = mouse_y + offset_y

    screen.fill("black")
    for ray in ray_list:
        hit = False
        bound_check = False

        for bound in bound_list:
            collision = ray.hit_boundary(bound)
            if collision and bound.name != ray.reflect_origin:
                if not hit:
                    hit = collision  # set initial hit
                    bound_check = bound
                else:  # if there are multiple hits, pick the least distance
                    dist_old = math.sqrt((hit[0] - ray_center[0]) ** 2 + (hit[1] - ray_center[1]) ** 2)
                    dist_new = math.sqrt((collision[0] - ray_center[0]) ** 2 + (collision[1] - ray_center[1]) ** 2)
                    if dist_new < dist_old:
                        hit = collision
                        bound_check = bound

        if hit:  # check if ray collision occured and if ray is being reflected twice from the previous bound
            pygame.draw.aaline(screen, (255, 0, 0), ray.pos, hit)
            """Reflection Code"""
            if bound_check.reflect:  # checks if hit is supposed to reflected
                normal_end = ray.normal_vector(bound_check, hit)
                normal_vec = (normal_end[0] - hit[0], normal_end[1] - hit[1])

                ray_vec = (ray.pos[0] - ray.end[0], ray.pos[1] - ray.end[1])
                reflect_vec = ray.reflect_vector(normal_vec, ray_vec)
                reflect_end = (hit[0] + 10 * reflect_vec[0], hit[1] + 10 * reflect_vec[1])  # translates the reflected vector direction by hit and enhances magnitude

                reflect_ray = Ray(hit, reflect_vec)
                reflect_ray.reflect_origin = bound_check.name
                ray_list.append(reflect_ray)


                # draw normal vector of the wall
                # pygame.draw.aaline(screen, (255, 0, 0), hit, normal_end)

                # draw reflected ray
                # pygame.draw.aaline(screen, (255, 0, 255), hit, reflect_end)

        else:
            if bound_check:
                # print(ray.reflect_origin, bound_check.name)
                pygame.draw.aaline(screen, (255, 255, 255), ray.pos,
                                   (ray.pos[0] + ray.dir[0] * 10, ray.pos[1] + ray.dir[1] * 10))

            else:
                pygame.draw.aaline(screen, (255, 0, 0), ray.pos,
                                   (ray.pos[0] + ray.dir[0] * 10, ray.pos[1] + ray.dir[1] * 10))

    pygame.draw.circle(surface, (255, 255, 0, 50), ray_center, radius + 20)
    screen.blit(surface, (0, 0))
    pygame.draw.circle(screen, (255, 255, 255), ray_center, radius)


    for wall in bound_list:
        pygame.draw.aaline(screen, (255, 255, 255), wall.start, wall.end)

    # flip() the display to put your work on screen
    pygame.display.flip()
    clock.tick(60)  # limits FPS to 60

pygame.quit()












