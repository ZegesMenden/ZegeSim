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
    
    def test_mul_scalar(self):
        self.assertEqual(vector3(1.0, 1.0, 1.0) * 2.0, vector3(2.0, 2.0, 2.0))

    def test_div(self):
        self.assertEqual(vector3(1.0, 1.0, 1.0) /
                         vector3(2.0, 2.0, 2.0), vector3(0.5, 0.5, 0.5))

    def test_div_scalar(self):
        self.assertEqual(vector3(1.0, 1.0, 1.0) / 2.0, vector3(0.5, 0.5, 0.5))

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
        self.assertEqual(quaternion(1.0, 0.0, 0.0, 0.0),
                         quaternion(1.0, 0.0, 0.0, 0.0))

    def test_mul(self):
        self.assertEqual(quaternion(1.0, 0.5, 0.5, 0.5) * quaternion(1.0,
                         0.5, 0.5, 0.5), quaternion(0.25, 1.0, 1.0, 1.0))

    def test_conj(self):
        self.assertEqual(quaternion(1.0, 0.5, 0.5, 0.5).conj(),
                         quaternion(1.0, -0.5, -0.5, -0.5))

    def test_norm(self):
        self.assertAlmostEqual(quaternion(1.0, 0.5, 0.5, 0.5).normalize().w, quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).w)

        self.assertAlmostEqual(quaternion(1.0, 0.5, 0.5, 0.5).normalize().x, quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).x)

        self.assertAlmostEqual(quaternion(1.0, 0.5, 0.5, 0.5).normalize().y, quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).y)

        self.assertAlmostEqual(quaternion(1.0, 0.5, 0.5, 0.5).normalize().z, quaternion(
            0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301).z)

    def test_euler_to_quaternion(self):
        
        e = vector3(45 * DEG_TO_RAD, 45 * DEG_TO_RAD, 45 * DEG_TO_RAD)
        q = quaternion().euler_to_quaternion(e)

        qt = quaternion(0.8446231020115715, 0.19134170284356303, 0.4619399539487806, 0.19134170284356303)

        self.assertAlmostEqual(q.w, qt.w, 4)
        self.assertAlmostEqual(q.x, qt.x, 4)
        self.assertAlmostEqual(q.y, qt.y, 4)
        self.assertAlmostEqual(q.z, qt.z, 4)

    def test_quaternion_to_euler(self):

        q = quaternion(0.8446231020115715, 0.19134170284356303, 0.4619399539487806, 0.19134170284356303)

        e = q.quaternion_to_euler()
        et = vector3(45 * DEG_TO_RAD, 45 * DEG_TO_RAD, 45 * DEG_TO_RAD)
        
        self.assertAlmostEqual(e.x, et.x, 4)
        self.assertAlmostEqual(e.y, et.y, 4)
        self.assertAlmostEqual(e.z, et.z, 4)

class test_physics_body(unittest.TestCase):

    def test_freefall(self):
        body = physics_body()

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
                               vector3(-9.8, 0.0, 0.0).x, 3)
        self.assertAlmostEqual(body.position.x,
                               vector3(-4.9, 0.0, 0.0).x, 3)

    def test_floor(self):

        body = physics_body()

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

        body = physics_body()

        body.floor = False
        body.mass = 1

        body.moment_of_inertia = vector3(1, 1, 1)

        time = 0.0
        body.rotation = quaternion().euler_to_quaternion(vector3(0.0, 0.0, 0.0))
        print(body.rotation.quaternion_to_euler().x)


        while True:

            body.add_torque(vector3(1.0, 0.0, 0.0))
            # print(body.rotation.quaternion_to_euler().x)
            body.update(0.00001)
            body.clear()

            if time >= 1.0:
                break

            time = time + 0.00001

        self.assertAlmostEqual(body.rotation.quaternion_to_euler().x, 0.5, 4)

    def test_local_toruqe(self):

        body = physics_body()

        body.floor = False
        body.mass = 1

        body.moment_of_inertia = vector3(1, 1, 1)

        time = 0.0
        body.rotation = quaternion().euler_to_quaternion(vector3(0.0, 0.0, 0.0))
        print(body.rotation.quaternion_to_euler().x)

        while True:

            body.add_torque_local(vector3(1.0, 0.0, 0.0))
            # print(body.rotation.quaternion_to_euler().x)
            body.update(0.00001)
            body.clear()

            if time >= 1.0:
                break

            time = time + 0.00001

        self.assertAlmostEqual(body.rotation.quaternion_to_euler().x, 0.5, 4)

class test_tvc(unittest.TestCase):

    def test_forces(self):

        tvc : TVC = TVC()

        tvc.servo_speed = 250
        tvc.linkage_ratio= 4
        tvc.max = 5
        tvc.min = -5
        tvc.position = vector3(0.0, 5, 5) * DEG_TO_RAD

        tvc.calculate_forces(10)

        self.assertAlmostEqual(tvc.force.x, 9.92389396, 5)
        self.assertAlmostEqual(tvc.force.y, 0.87155742, 5)
        self.assertAlmostEqual(tvc.force.z, 0.87155742, 5)

if __name__ == '__main__':
    unittest.main()
