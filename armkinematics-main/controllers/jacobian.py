import numpy as np
import math
from sympy import Symbol, Matrix, cos, sin, diff

MAX_STEP = 10

class JacobianController:
    def __init__(self, arm):
            self.arm = arm

            theta1 = Symbol("theta1")
            theta2 = Symbol("theta2")
            theta3 = Symbol("theta3")
            theta4 = Symbol("theta4")
            theta5 = Symbol("theta5")
            theta6 = Symbol("theta6")

            l = 40

            T1 = Matrix([
                [cos(theta1), -sin(theta1), 0, l * cos(theta1)],
                [sin(theta1), cos(theta1), 0, l * sin(theta1)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

            T2 = Matrix([
                [cos(theta2), -sin(theta2), 0, l * cos(theta2)],
                [sin(theta2), cos(theta2), 0, l * sin(theta2)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

            T3 = Matrix([
                [cos(theta3), -sin(theta3), 0, l * cos(theta3)],
                [sin(theta3), cos(theta3), 0, l * sin(theta3)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

            T4 = Matrix([
                [cos(theta4), -sin(theta4), 0, l * cos(theta4)],
                [sin(theta4), cos(theta4), 0, l * sin(theta4)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

            T5 = Matrix([
                [cos(theta5), -sin(theta5), 0, l * cos(theta5)],
                [sin(theta5), cos(theta5), 0, l * sin(theta5)],
                [0, 0, 1, 0],
                [0, 0, 0, 1] ])

            T6 = Matrix([
                [cos(theta6), -sin(theta6), 0, l * cos(theta6)],
                [sin(theta6), cos(theta6), 0, l * sin(theta6)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

            T = T1 * T2 * T3 * T4 * T5 * T6

            x = T[3]
            y = T[7]

            a00 = diff(x, theta1)
            a01 = diff(x, theta2)
            a02 = diff(x, theta3)
            a03 = diff(x, theta4)
            a04 = diff(x, theta5)
            a05 = diff(x, theta6)

            a10 = diff(y, theta1)
            a11 = diff(y, theta2)
            a12 = diff(y, theta3)
            a13 = diff(y, theta4)
            a14 = diff(y, theta5)
            a15 = diff(y, theta6)

            self.J_simbolico = Matrix([
                [a00, a01, a02, a03, a04, a05],
                [a10, a11, a12, a13, a14, a15]
            ])

            self.simbolos = [
                theta1, theta2, theta3,
                theta4, theta5, theta6
            ]

            a00 = a00.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a01 = a01.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a02 = a02.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a03 = a03.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a04 = a04.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a05 = a05.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a10 = a10.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a11 = a11.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a12 = a12.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a13 = a13.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a14 = a14.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])
            a15 = a15.subs(
                [(theta1, 0), (theta2, math.radians(90)), (theta3, 0), (theta4, 0), (theta5, 0), (theta6, 0)])

    def control(self, target):
        if self.arm.get_num_joints() == 2:
            self.control2J2D(target)
        elif self.arm.get_num_joints() == 3:
            self.control3J2D(target)
        elif self.arm.get_num_joints() == 6:
            self.control6J2D(target)
        else:
            raise Exception("JacobianController.control(target): Can't control an arm with this amount joints.")

    def control2J2D(self, target):
        # the control method receives a target
        pass


    def control3J2D(self, target):
        # the control method receives a target
        pass

    def control6J2D(self, target):
        sustituciones = [
            (self.simbolos[0], self.arm.thetas[0]),
            (self.simbolos[1], self.arm.thetas[1]),
            (self.simbolos[2], self.arm.thetas[2]),
            (self.simbolos[3], self.arm.thetas[3]),
            (self.simbolos[4], self.arm.thetas[4]),
            (self.simbolos[5], self.arm.thetas[5])
        ]

        J = np.array(
            self.J_simbolico.subs(sustituciones),
            dtype=float
        )

        error = target - self.arm.endeffector()
        paso = min(error.r, MAX_STEP)

        delta_c = np.array([
            paso * math.cos(error.a),
            paso * math.sin(error.a)
        ])

        J_pinv = np.linalg.pinv(J)
        delta_ang = J_pinv.dot(delta_c)

        self.arm.move(delta_ang)