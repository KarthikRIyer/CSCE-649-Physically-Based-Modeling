import taichi as ti
import numpy as np

ti.init(arch=ti.cpu, debug=True)
# ti.init(arch=ti.gpu)

# GUI
WIDTH = 600
HEIGHT = 600
BACKGROUND_COLOUR = 0xf0f0f0
PARTICLE_COLOUR = 0x328ac1
PARTICLE_RADIUS = 4
dt = 1 / 60

NUM_PARTICLES = 1000

x = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
# x_temp = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
v = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
alignSteering = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
cohesionSteering = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
separationSteering = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
# v_temp = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
a = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
a_temp = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)
x_display = ti.Vector.field(2, dtype=ti.f32, shape=NUM_PARTICLES)

vMag = 200.0
perceptionDist = 50
maxForce = 400.0
maxSpeed = vMag

alignFactor = 1.2
cohesionFactor = 1.0
separationFactor = 1.1

collisionPenaltyForceMag = 1000.0

@ti.kernel
def setup():
 for i in range(NUM_PARTICLES):
     x[i] = ti.random(dtype=float) * WIDTH, ti.random(dtype=float) * HEIGHT
     v[i] = (ti.random(dtype=float) * 2.0 - 1.0), (ti.random(dtype=float) * 2.0 -1)
     v[i] = ti.math.normalize(v[i])
     v[i] *= vMag
     a[i] = 0.0, 0.0


@ti.kernel
def apply_forces():
    for i in range(NUM_PARTICLES):
        x[i] = x[i] + dt * v[i]
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


def simulate():
    align()
    cohesion()
    separation()
    calc_net_acceleration()
    apply_forces()
    update()


@ti.kernel
def update():
    for i in range(NUM_PARTICLES):
        x_display[i][0] = x[i][0] / WIDTH
        x_display[i][1] = x[i][1] / HEIGHT


def render(gui):
    q = x_display.to_numpy()
    for i in range(NUM_PARTICLES):
        gui.circle(pos=q[i], color=PARTICLE_COLOUR, radius=PARTICLE_RADIUS)
    gui.show()


if __name__ == '__main__':
    gui = ti.GUI('Flocking', res=(WIDTH, HEIGHT), background_color=BACKGROUND_COLOUR)
    setup()
    while True:
        simulate()
        render(gui)
