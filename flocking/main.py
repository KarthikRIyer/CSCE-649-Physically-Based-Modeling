import taichi as ti
import numpy as np

# ti.init(arch=ti.cpu, debug=True)
ti.init(arch=ti.cpu)
# ti.init(arch=ti.gpu)

# GUI
WIDTH = 600
HEIGHT = 600
BACKGROUND_COLOUR = 0xf0f0f0
PARTICLE_COLOUR = 0x328ac1
GOAL_COLOR = 0x00ff00
PARTICLE_RADIUS = 4
dt = 1 / 60

NUM_PARTICLES = 1000
NUM_GOALS = 1

x = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
xp1 = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
xp2 = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
xp3 = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
# x_temp = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
v = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
alignSteering = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
cohesionSteering = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
goalSeekSteering = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
separationSteering = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
# v_temp = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
a = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
a_temp = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
x_display = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
xp1_display = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
xp2_display = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
xp3_display = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)

goalX = ti.Vector.field(2, dtype=ti.f32, shape=NUM_GOALS)
goalXDisplay = ti.Vector.field(2, dtype=ti.f32, shape=NUM_GOALS)

vMag = 200.0
perceptionDist = 50
maxForce = 400.0
maxSpeed = vMag

alignFactor = 1.2
cohesionFactor = 1.2
separationFactor = 1.3
goalSeekFactor = 0.2

collisionPenaltyForceMag = 1000.0
t = 0.0


@ti.kernel
def setup():
    goalX[0] = 300, 300
    for i in range(NUM_PARTICLES):
        x[i] = ti.random(dtype=float) * WIDTH, ti.random(dtype=float) * HEIGHT
        v[i] = (ti.random(dtype=float) * 2.0 - 1.0), (ti.random(dtype=float) * 2.0 -1)
        # v[i] = (1, -1)
        v[i] = ti.math.normalize(v[i])
        v[i] *= vMag
        a[i] = 0.0, 0.0
        xp1[i] = 0.0, 0.0
        xp2[i] = 0.0, 0.0
        xp3[i] = 0.0, 0.0


@ti.kernel
def apply_forces():
    for i in range(NUM_PARTICLES):
        x[i] = x[i] + dt * v[i]

        angle = ti.math.atan2(v[i][1], v[i][0])
        angle -= 1.5708
        # print(angle)
        cos_angle = ti.math.cos(angle)
        sin_angle = ti.math.sin(angle)
        xp1[i] = x[i] + ti.Vector([0, PARTICLE_RADIUS * 4])
        xp1[i] = x[i] + ti.Vector(
            [cos_angle * (xp1[i][0] - x[i][0]) - sin_angle * (xp1[i][1] - x[i][1]),
             sin_angle * (xp1[i][0] - x[i][0]) + cos_angle * (xp1[i][1] - x[i][1])])
        xp2[i] = x[i] + ti.Vector([PARTICLE_RADIUS * 0.866, -PARTICLE_RADIUS * 0.5])
        xp2[i] = x[i] + ti.Vector(
            [cos_angle * (xp2[i][0] - x[i][0]) - sin_angle * (xp2[i][1] - x[i][1]),
             sin_angle * (xp2[i][0] - x[i][0]) + cos_angle * (xp2[i][1] - x[i][1])])
        xp3[i] = x[i] + ti.Vector([-PARTICLE_RADIUS * 0.866, -PARTICLE_RADIUS * 0.5])
        xp3[i] = x[i] + ti.Vector(
            [cos_angle * (xp3[i][0] - x[i][0]) - sin_angle * (xp3[i][1] - x[i][1]),
             sin_angle * (xp3[i][0] - x[i][0]) + cos_angle * (xp3[i][1] - x[i][1])])

        v[i] = v[i] + dt * a[i]
        if ti.math.length(v[i]) > maxSpeed:
            v[i] = maxSpeed * ti.math.normalize(v[i])

        a[i] = 0, 0

        if x[i][0] > WIDTH:
            # x[i][0] = 0
            a[i][0] = -collisionPenaltyForceMag
        elif x[i][0] < 0:
            # x[i][0] = WIDTH
            a[i][0] = collisionPenaltyForceMag

        if x[i][1] > HEIGHT:
            # x[i][1] = 0
            a[i][1] = -collisionPenaltyForceMag
        elif x[i][1] < 0:
            # x[i][1] = HEIGHT
            a[i][1] = collisionPenaltyForceMag
        # a[i] = ti.math.normalize(a[i])
        # a[i] *= maxForce


@ti.kernel
def calc_net_acceleration():
    for i in range(NUM_PARTICLES):
        # a[i] = 0, 0
        a[i] += alignFactor * alignSteering[i]
        a[i] += cohesionFactor * cohesionSteering[i]
        a[i] += separationFactor * separationSteering[i]
        a[i] += goalSeekFactor * goalSeekSteering[i]
        # print(a[i])


@ti.kernel
def align():
    for i in range(NUM_PARTICLES):
        alignSteering[i] = 0.0, 0.0
        totalNeigh = 0
        for j in range(NUM_PARTICLES):
            if i == j:
                continue
            d = ti.math.length(x[j] - x[i])
            if d < perceptionDist:
                totalNeigh += 1
                alignSteering[i] += v[j]
        if totalNeigh > 0:
            # alignSteering[i] /= totalNeigh
            alignSteering[i] = ti.math.normalize(alignSteering[i])
            alignSteering[i] *= maxSpeed
            alignSteering[i] -= v[i]
            # print(ti.math.length(alignSteering[i]))
            if ti.math.length(alignSteering[i]) > maxForce:
                alignSteering[i] = ti.math.normalize(alignSteering[i])
                alignSteering[i] *= maxForce


@ti.kernel
def cohesion():
    for i in range(NUM_PARTICLES):
        cohesionSteering[i] = 0.0, 0.0
        totalNeigh = 0
        for j in range(NUM_PARTICLES):
            if i == j:
                continue
            d = ti.math.length(x[j] - x[i])
            if d < perceptionDist:
                totalNeigh += 1
                cohesionSteering[i] += x[j]
        if totalNeigh > 0:
            cohesionSteering[i] /= totalNeigh
            cohesionSteering[i] -= x[i]
            cohesionSteering[i] = ti.math.normalize(cohesionSteering[i])
            cohesionSteering[i] *= maxSpeed
            cohesionSteering[i] -= v[i]
            # print(ti.math.length(cohesionSteering[i]))
            if ti.math.length(cohesionSteering[i]) > maxForce:
                cohesionSteering[i] = ti.math.normalize(cohesionSteering[i])
                cohesionSteering[i] *= maxForce


@ti.kernel
def goalSeek():
    for i in range(NUM_PARTICLES):
        goalSeekSteering[i] = 0.0, 0.0
        for j in range(NUM_GOALS):
            goalSeekSteering[i] += goalX[j]
        goalSeekSteering[i] /= NUM_GOALS
        goalSeekSteering[i] -= x[i]
        goalSeekSteering[i] = ti.math.normalize(goalSeekSteering[i])
        goalSeekSteering[i] *= maxSpeed
        goalSeekSteering[i] -= v[i]
        # print(ti.math.length(goalSeekSteering[i]))
        if ti.math.length(goalSeekSteering[i]) > maxForce:
            goalSeekSteering[i] = ti.math.normalize(goalSeekSteering[i])
            goalSeekSteering[i] *= maxForce


@ti.kernel
def separation():
    for i in range(NUM_PARTICLES):
        separationSteering[i] = 0.0, 0.0
        totalNeigh = 0
        for j in range(NUM_PARTICLES):
            if i == j:
                continue
            d = ti.math.length(x[j] - x[i])
            if d < perceptionDist:
                totalNeigh += 1
                diff = x[i] - x[j]
                diff *= (1.0 / (d * d))
                separationSteering[i] += diff
        if totalNeigh > 0:
            separationSteering[i] /= totalNeigh
            separationSteering[i] = ti.math.normalize(separationSteering[i])
            separationSteering[i] *= maxSpeed
            separationSteering[i] -= v[i]
            # print(ti.math.length(separationSteering[i]))
            if ti.math.length(separationSteering[i]) > maxForce:
                separationSteering[i] = ti.math.normalize(separationSteering[i])
                separationSteering[i] *= maxForce


@ti.kernel
def move_goal(time: ti.f32):
    av = 0.5
    theta = av * time
    # print(theta)
    vel = ti.Vector([-ti.math.sin(theta), ti.math.cos(theta)]) * av * 100.0
    # print(vel)
    goalX[0] += vel * dt
    # print(goalX[0])


def simulate():
    global t
    move_goal(t)
    align()
    cohesion()
    separation()
    goalSeek()
    calc_net_acceleration()
    apply_forces()
    update()
    t += dt


@ti.kernel
def update():
    for i in range(NUM_GOALS):
        goalXDisplay[i][0] = goalX[i][0] / WIDTH
        goalXDisplay[i][1] = goalX[i][1] / HEIGHT
    for i in range(NUM_PARTICLES):
        x_display[i][0] = x[i][0] / WIDTH
        x_display[i][1] = x[i][1] / HEIGHT

        xp1_display[i][0] = xp1[i][0] / WIDTH
        xp1_display[i][1] = xp1[i][1] / HEIGHT
        xp2_display[i][0] = xp2[i][0] / WIDTH
        xp2_display[i][1] = xp2[i][1] / HEIGHT
        xp3_display[i][0] = xp3[i][0] / WIDTH
        xp3_display[i][1] = xp3[i][1] / HEIGHT


def render(gui):
    q = x_display.to_numpy()
    p1 = xp1_display.to_numpy()
    p2 = xp2_display.to_numpy()
    p3 = xp3_display.to_numpy()
    for i in range(NUM_PARTICLES):
        # gui.circle(pos=q[i], color=PARTICLE_COLOUR, radius=PARTICLE_RADIUS)
        gui.triangle(p1[i], p2[i], p3[i], color=PARTICLE_COLOUR)
    q = goalXDisplay.to_numpy()
    for i in range(NUM_GOALS):
        gui.circle(pos=q[i], color=GOAL_COLOR, radius=PARTICLE_RADIUS * 2)
    gui.show()


if __name__ == '__main__':
    gui = ti.GUI('Flocking', res=(WIDTH, HEIGHT), background_color=BACKGROUND_COLOUR)
    setup()
    while True:
        simulate()
        render(gui)
