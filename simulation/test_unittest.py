from physics import *
import unittest


class test_vector_math(unittest.TestCase):

    def test_eq(self):
        self.assertEqual(vector3(0.0, 0.0, 0.0), vector3(0.0, 0.0, 0.0))

    def test_add(self):
        self.assertEqual(vector3(0.0, 0.0, 0.0) +
                         vector3(1.0, 1.0, 1.0), vector3(1.0, 1.0, 1.0))

    def test_sub(self):
        self.assertEqual(vector3(0.0, 0.0, 0.0) - vector3(1.0,
                         1.0, 1.0), vector3(-1.0, -1.0, -1.0))

    def test_mul(self):
        self.assertEqual(vector3(1.0, 1.0, 1.0) *
                         vector3(2.0, 2.0, 2.0), vector3(2.0, 2.0, 2.0))

    def test_div(self):
        self.assertEqual(vector3(1.0, 1.0, 1.0) /
                         vector3(2.0, 2.0, 2.0), vector3(0.5, 0.5, 0.5))

    def test_norm(self):
        self.assertEqual(vector3(5.0, 5.0, 5.0).normalize(), vector3(
            0.5773502691896257, 0.5773502691896257, 0.5773502691896257))

    def test_len(self):
        self.assertEqual(vector3(5.0, 5.0, 5.0).norm(), 8.660254037844387)

    def test_abs(self):
        self.assertEqual(vector3(1.0, 1.0, 1.0),
                         abs(vector3(-1.0, -1.0, -1.0)))

    def test_round(self):
        self.assertEqual(vector3(1.1, 1.1, 1.1),
                         round(vector3(1.11, 1.11, 1.11), 1))

    def test_str(self):
        self.assertEqual("1.0, 1.0, 1.0", str(vector3(1.0, 1.0, 1.0)))


class test_quaternion_math(unittest.TestCase):

    def test_eq(self):
        self.assertEqual(Quaternion(1.0, 0.0, 0.0, 0.0),
                         Quaternion(1.0, 0.0, 0.0, 0.0))

    def test_add(self):
        self.assertEqual((Quaternion(0.0, 0.0, 0.0, 0.0) + Quaternion(1.0,
                         1.0, 1.0, 1.0)).w, Quaternion(1.0, 1.0, 1.0, 1.0).w)

    def test_sub(self):
        self.assertEqual((Quaternion(1.0, 1.0, 1.0, 1.0) - Quaternion(1.0,
                         1.0, 1.0, 1.0)).w, Quaternion(0.0, 0.0, 0.0, 0.0).w)

    def test_mul(self):
        self.assertEqual(Quaternion(1.0, 0.5, 0.5, 0.5) * Quaternion(1.0,
                         0.5, 0.5, 0.5), Quaternion(0.25, 1.0, 1.0, 1.0))

    def test_conj(self):
        self.assertEqual(Quaternion(1.0, 0.5, 0.5, 0.5).conj(),
                         Quaternion(1.0, -0.5, -0.5, -0.5))

    def test_norm(self):
        self.assertAlmostEqual(Quaternion(1.0, 0.5, 0.5, 0.5).norm().w, Quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).w)

        self.assertAlmostEqual(Quaternion(1.0, 0.5, 0.5, 0.5).norm().x, Quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).x)

        self.assertAlmostEqual(Quaternion(1.0, 0.5, 0.5, 0.5).norm().y, Quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).y)

        self.assertAlmostEqual(Quaternion(1.0, 0.5, 0.5, 0.5).norm().z, Quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).z)

    def test_euler_to_quaternion(self):
        self.assertAlmostEqual(Quaternion().eulerToQuaternion(vector3(0.174533, 0.174533, 0.174533)).w, Quaternion(
            0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543).w)

        self.assertAlmostEqual(Quaternion().eulerToQuaternion(vector3(0.174533, 0.174533, 0.174533)).x, Quaternion(
            0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543).x)

        self.assertAlmostEqual(Quaternion().eulerToQuaternion(vector3(0.174533, 0.174533, 0.174533)).y, Quaternion(
            0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543).y)

        self.assertAlmostEqual(Quaternion().eulerToQuaternion(vector3(0.174533, 0.174533, 0.174533)).z, Quaternion(
            0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543).z)

    def test_quaternion_to_euler(self):
        self.assertAlmostEqual(Quaternion(0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543).quaternionToEuler().x,
                               vector3(0.1745, 0.1736, 0.1745).x, 4)

        self.assertAlmostEqual(Quaternion(0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543).quaternionToEuler().y,
                               vector3(0.1745, 0.1736, 0.1745).y, 4)

        self.assertAlmostEqual(Quaternion(0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543).quaternionToEuler().z,
                               vector3(0.1745, 0.1736, 0.1745).z, 4)


class test_physics_body(unittest.TestCase):

    def test_freefall(self):
        body = physicsBody()

        body.floor = False
        body.mass = 1
        body.gravity = vector3(-9.8, 0.0, 0.0)

        time = 0.0

        while True:
            body.update(0.00001)
            body.clear()
            time += 0.00001
            if time > 1.0:
                break

        self.assertAlmostEqual(body.velocity.x,
                               vector3(-9.8, 0.0, 0.0).x, 2)
        self.assertAlmostEqual(body.position.x,
                               vector3(-4.9, 0.0, 0.0).x, 2)
        # 4.9 because of the way the sim works, this isnt an issue ( well it sorta is but its not because of bad math its because of a bad sim)

    def test_floor(self):

        body = physicsBody()

        body.floor = True
        body.mass = 1
        body.gravity = vector3(-9.8, 0.0, 0.0)
        body.position = vector3(1.0, 1.0, 1.0)

        dt = 0.01
        time = 0.0
        while time < 2.0:
            body.update(dt)
            body.clear()
            time += dt        

        self.assertEqual(body.velocity, vector3(0.0, 0.0, 0.0))
        self.assertEqual(body.position, vector3(0.0, 1.0, 1.0))

    def test_toruqe(self):

        body = physicsBody()

        body.floor = False
        body.mass = 1

        body.moment_of_inertia = vector3(1, 1, 1)

        time = 0.0

        while True:

            body.addTorque(vector3(1.0, 0.0, 0.0))
            body.update(0.01)
            body.clear()

            if time >= 1.0:
                break

            time = time + 0.01

        self.assertAlmostEqual(body.rotation_euler.x, 0.5, 1)

    def test_local_toruqe(self):

        body = physicsBody()

        body.floor = False
        body.mass = 1

        body.moment_of_inertia = vector3(1, 1, 1)

        body.rotation_euler = vector3(45.0, 0.0, 0.0) * DEG_TO_RAD
        body.rotation_quaternion = body.rotation_quaternion.eulerToQuaternion(body.rotation_euler)

        time = 0.0

        while True:

            body.addTorqueLocal(vector3(0.0, 1.0, 0.0))
            body.update(0.01)
            body.clear()

            if time >= 1.0:
                break

            time = time + 0.01

            # print(body.rotation_euler * RAD_TO_DEG)

        self.assertAlmostEqual(body.rotation_euler.z, 0.5, 2)

class test_tvc(unittest.TestCase):

    def test_forces(self):

        tvc : TVC = TVC()

        tvc.servoSpeed = 250
        tvc.linkageRatio= 4
        tvc.max = 5
        tvc.min = -5
        tvc.position = vector3(0.0, 5, 5) * DEG_TO_RAD

        tvc.calculateForces(10)

        self.assertAlmostEqual(tvc.force.x, 9.92389396, 5)
        self.assertAlmostEqual(tvc.force.y, 0.87155742, 5)
        self.assertAlmostEqual(tvc.force.z, 0.87155742, 5)


if __name__ == '__main__':
    unittest.main()
