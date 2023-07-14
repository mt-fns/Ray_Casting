import math
import pygame
import numpy as np

bound_list = []
degree = 10 # degree between rays
ray_center = [500, 200]
radius = 20
bound_width = 4

# display reflection number (r = 1/ n = 1)

def rotate_vector(x, y, rad):
    return (x * math.cos(rad) - y * math.sin(rad), x * math.sin(rad) + y * math.cos(rad))

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return np.array(vector) / np.linalg.norm(vector)

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
        self.magnitude = 0  # used for animating ray lines that collide with bounds
        self.hit = None  # name of boundary that ray hits
        self.parent_ray = None  # the ray it was reflected from
        self.done_animating = False

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

    def animate_ray(self, increment):

        if self.hit:
            if self.pos[0] + self.dir[0] - self.pos[0] > 0:  # if direction is positive
                if self.pos[0] + self.dir[0] * self.magnitude < self.hit[0]:
                    self.magnitude += increment

                    self.end = (self.pos[0] + self.dir[0] * self.magnitude, self.pos[1] + self.dir[1] * self.magnitude)
                else:
                    self.done_animating = True

            elif self.pos[0] + self.dir[0] - self.pos[0] < 0:  # if direction is negative
                if self.pos[0] + self.dir[0] * self.magnitude > self.hit[0]:
                    self.magnitude += increment

                    self.end = (self.pos[0] + self.dir[0] * self.magnitude, self.pos[1] + self.dir[1] * self.magnitude)
                else:
                    self.done_animating = True

            elif self.pos[0] + self.dir[0] - self.pos[0] == 0:  # if direction is zero (no changes in x)
                if self.dir[1] > 0:
                    if self.pos[1] + self.dir[1] * self.magnitude < self.hit[1]:
                        self.magnitude += increment

                        self.end = (self.pos[0] + self.dir[0] * self.magnitude, self.pos[1] + self.dir[1] * self.magnitude)
                    else:
                        self.done_animating = True

                elif self.dir[1] < 0:
                    if self.pos[1] + self.dir[1] * self.magnitude > self.hit[1]:
                        self.magnitude += increment

                        self.end = (self.pos[0] + self.dir[0] * self.magnitude, self.pos[1] + self.dir[1] * self.magnitude)
                    else:
                        self.done_animating = True

        else:
            self.magnitude += increment

            self.end = (self.pos[0] + self.dir[0] * self.magnitude, self.pos[1] + self.dir[1] * self.magnitude)

# bug with this config, i aint fixin it lmaoooo
# bound1 = Wall((300, 100), (200, 300), "1")
# bound2 = Wall((700, 100), (800, 300), "2")
# bound3 = Wall((300, 500), (700, 600), "3")
# bound3.reflect = True
# bound2.reflect = True
# bound_list.append(bound2)
# bound_list.append(bound1)
# bound_list.append(bound3)

bound1 = Wall((300, 100), (200, 300), "1")
bound2 = Wall((700, 100), (800, 300), "2")
bound3 = Wall((800, 300), (500, 600), "3")
bound4 = Wall((200, 300), (500, 600), "4")
bound5 = Wall((300, 100), (400, 100), "5")
bound6 = Wall((700, 100), (600, 100), "6")
bound7 = Wall((400, 100), (500, 200), "7")
bound8 = Wall((600, 100), (500, 200), "8")
bound5.reflect = True
bound6.reflect = True
bound3.reflect = True
bound4.reflect = True

bound_list.append(bound1)
bound_list.append(bound2)
bound_list.append(bound3)
bound_list.append(bound4)
bound_list.append(bound5)
bound_list.append(bound6)
bound_list.append(bound7)
bound_list.append(bound8)













pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dragging = False
offset_x = 0
offset_y = 0
start_casting = False
ray_frames = 100
magnitude = 0  # used to iterate and update frames of rays
ray_list = None

while running:
    screen.fill("black")

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif not start_casting:
             if event.type == pygame.MOUSEBUTTONDOWN:
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

             elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    start_casting = True

    if start_casting:
        if not ray_list:  # create list of rays
            ray1 = Ray(ray_center, (100, 0))  # initial ray to be used in rendering the rest
            ray_list = create_rays(ray1)
            for ray in ray_list:
                bound_check = False

                for bound in bound_list:
                    collision = ray.hit_boundary(bound)
                    if collision and bound.name != ray.reflect_origin:
                        if not ray.hit:
                            ray.hit = collision  # set initial hit
                            bound_check = bound
                        else:  # if there are multiple hits, pick the least distance
                            dist_old = math.sqrt((ray.hit[0] - ray_center[0]) ** 2 + (ray.hit[1] - ray_center[1]) ** 2)
                            dist_new = math.sqrt(
                                (collision[0] - ray_center[0]) ** 2 + (collision[1] - ray_center[1]) ** 2)
                            if dist_new < dist_old:
                                ray.hit = collision
                                bound_check = bound

                if ray.hit:  # check if ray collision occured
                    hit_dir = (ray.hit[0] - ray.pos[0], ray.hit[1] - ray.pos[1])
                    hit_vec = unit_vector(hit_dir)

                    """Reflection Code"""
                    if bound_check.reflect:  # checks if hit is supposed to reflected
                        normal_end = ray.normal_vector(bound_check, ray.hit)
                        normal_vec = (normal_end[0] - ray.hit[0], normal_end[1] - ray.hit[1])

                        ray_vec = (ray.pos[0] - ray.end[0], ray.pos[1] - ray.end[1])
                        reflect_vec = ray.reflect_vector(normal_vec, ray_vec)
                        reflect_end = (ray.hit[0] + 10 * reflect_vec[0], ray.hit[1] + 10 * reflect_vec[
                            1])  # translates the reflected vector direction by hit and enhances magnitude

                        reflect_ray = Ray(ray.hit, reflect_vec)
                        reflect_ray.reflect_origin = bound_check.name
                        reflect_ray.parent_ray = ray
                        ray_list.append(reflect_ray)

        else:
            for ray in ray_list:
                if not ray.parent_ray:
                    if not ray.done_animating:
                        ray.animate_ray(0.01)
                    pygame.draw.aaline(screen, (255, 0, 0), ray.pos, ray.end)


                elif ray.parent_ray.done_animating:
                    if not ray.done_animating:
                        ray.animate_ray(0.01)
                    pygame.draw.aaline(screen, (255,20,147), ray.pos, ray.end)


    # flip() the display to put your work on screen
    surface = pygame.Surface((screen.get_width(), screen.get_height()), pygame.SRCALPHA)

    for wall in bound_list:
        pygame.draw.line(screen, (255, 255, 255), wall.start, wall.end, width=bound_width)

    pygame.draw.circle(surface, (255, 0, 0, 50), ray_center, radius + 20)
    screen.blit(surface, (0, 0))
    pygame.draw.circle(screen, (255, 255, 255), ray_center, radius)

    pygame.display.flip()
    clock.tick(60)  # limits FPS to 60

pygame.quit()