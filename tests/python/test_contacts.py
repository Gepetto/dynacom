#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 16:28:36 2022

@author: nvilla
"""
import aig
import dynacom
import numpy as np
from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR
import unittest
import pinocchio as pin

# from time import sleep
import matplotlib.pyplot as plt

from correct_values import correct_left_z_force, correct_right_z_force

unittest.util._MAX_LENGTH = 2000

debug = False
visualize = False


class TestDynaCoM(unittest.TestCase):
    def setUp(self):

        # CONTACTS ##
        settingsL = dynacom.Contact6DSettings()
        settingsL.frame_name = "leg_left_sole_fix_joint"
        settingsL.mu = 0.3
        settingsL.gu = 0.4
        settingsL.weights = np.array([1, 1, 1, 1, 1, 1])
        settingsL.half_length = 0.1
        settingsL.half_width = 0.05

        leftSole = dynacom.Contact6D()
        leftSole.initialize(settingsL)

        settingsR = dynacom.Contact6DSettings()
        settingsR.frame_name = "leg_right_sole_fix_joint"
        settingsR.mu = 0.3
        settingsR.gu = 0.4
        settingsR.weights = np.array([1, 1, 1, 1, 1, 1])
        settingsR.half_length = 0.1
        settingsR.half_width = 0.05

        rightSole = dynacom.Contact6D()
        rightSole.initialize(settingsR)

        # DYNAMICS ###
        dynSettings = dynacom.DynaCoMSettings()
        dynSettings.urdf = (
            EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data/robots/talos_reduced.urdf"
        )

        d = dynacom.DynaCoM()
        d.initialize(dynSettings)

        pin.loadReferenceConfigurations(
            d.model(),
            EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data/srdf/talos.srdf",
            False,
        )

        q0 = d.model().referenceConfigurations["half_sitting"]
        v0 = np.zeros(d.model().nv)
        d.computeDynamics(q0, v0, v0, np.zeros(6), True)
        pin.computeAllTerms(d.model(), d.data(), q0, v0)
        pin.updateFramePlacements(d.model(), d.data())

        d.addContact6d(leftSole, "left_sole")
        d.addContact6d(rightSole, "right_sole")

        # BIPED IG to compute postures
        settings = aig.BipedIGSettings.makeSettingsFor(
            EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data", "talos"
        )
        biped = aig.BipedIG()
        biped.initialize(settings)

        self.dyn = d
        self.setL = settingsL
        self.setR = settingsR
        self.q0 = q0
        self.biped = biped

    def test_contacts(self):

        leftSole = self.dyn.getContact("left_sole")

        self.assertTrue(leftSole.get_settings()["mu"] == self.setL.mu)
        leftSole.set_mu(0.6)
        self.assertTrue(leftSole.get_settings()["mu"] == 0.6)

        self.assertTrue((leftSole.get_settings()["weights"] == self.setL.weights).all())
        leftSole.set_force_weights(np.array([3, 3, 3]))
        self.assertTrue(
            (leftSole.get_settings()["weights"][:3] == np.array([3, 3, 3])).all()
        )
        leftSole.set_torque_weights(np.array([3, 3, 3]))
        self.assertTrue(
            (leftSole.get_settings()["weights"][3:] == np.array([3, 3, 3])).all()
        )
        self.assertTrue(
            list(self.dyn.getActiveContacts()) == ["left_sole", "right_sole"]
        )
        self.dyn.deactivateContact6d("left_sole")
        self.assertTrue("left_sole" not in list(self.dyn.getActiveContacts()))

        self.dyn.deactivateContact6d("right_sole")
        self.assertTrue(not list(self.dyn.getActiveContacts()))

        self.dyn.activateContact6d("left_sole")
        self.assertTrue("left_sole" in list(self.dyn.getActiveContacts()))

        self.dyn.activateContact6d("right_sole")
        self.assertTrue(
            list(self.dyn.getActiveContacts()) == ["left_sole", "right_sole"]
        )
        self.dyn.removeContact6d("right_sole")
        self.assertTrue("right_sole" not in list(self.dyn.getActiveContacts()))

    def test_adjoint(self):

        H1 = pin.SE3(np.eye(3), np.array([0, 1, 0]))
        Adj1 = H1.toActionMatrixInverse().T

        W1l = np.array([0, 0, 1, 0, 0, 0])
        W1o_correct = np.array([0, 0, 1, 1, 0, 0])

        self.assertTrue((Adj1 @ W1l == W1o_correct).all())

        H2 = pin.SE3(pin.utils.rotate("y", 0.3), np.array([0, 1, 0]))
        Adj2 = H2.toActionMatrixInverse().T

        W2l = np.array([0, 0, 0, 1, 0, 0])
        W2o_correct = np.hstack([0, 0, 0, H2.rotation @ [1, 0, 0]])

        self.assertTrue((Adj2 @ W2l == W2o_correct).all())

    def test_distribution_on_single_contact(self):

        self.dyn.deactivateContact6d("right_sole")
        pin.updateFramePlacements(self.dyn.model(), self.dyn.data())
        data = self.dyn.data()
        oMs = data.oMf[self.dyn.getContact("left_sole").get_frame_id()]
        com = oMs.translation + np.array([0, -0.2, 1])

        cMo = pin.SE3(np.eye(3), -com)
        cXs = (cMo.act(oMs)).toActionMatrixInverse().T

        sMs = pin.SE3(oMs.rotation.T, np.zeros(3))
        correct_lW = sMs.toActionMatrixInverse().T @ np.array([0, 0, 10000, 0, 0, 0])

        cW = cXs @ correct_lW
        self.assertTrue((cW - np.array([0, 0, 10000, 2000, 0, 0]) < 1e-3).all())

        self.dyn.distributeForce(cW[:3], cW[3:], com)

        lW = self.dyn.getContact("left_sole").appliedForce()
        self.assertTrue((np.abs(lW - correct_lW) < 1e-4).all())

    def test_distribution_on_double_contact(self):

        com = np.array([0, 0, 2])
        cW = np.array([0, 0, 1, 0.01, 0, 0])

        self.dyn.distributeForce(cW[:3], cW[3:], com)
        self.assertTrue(
            (
                self.dyn.getContact("left_sole").appliedForce()[:3]
                + self.dyn.getContact("right_sole").appliedForce()[:3]
                - cW[:3]
                < 1e-4
            ).all()
        )

        self.assertTrue(
            self.dyn.getContact("left_sole").appliedForce()[2]
            > self.dyn.getContact("right_sole").appliedForce()[2]
        )

    def test_dynamic_distribution(self):

        LF = self.dyn.getContact("left_sole").get_pose()
        RF = self.dyn.getContact("right_sole").get_pose()
        N_cycles = 5
        N = 500
        Dt = 0.01

        self.dyn.getContact("left_sole").set_force_weights(np.array([1e-5, 1e-5, 1e-5]))
        self.dyn.getContact("right_sole").set_force_weights(
            np.array([1e-5, 1e-5, 1e-5])
        )

        time = np.linspace(0, N, N) * N_cycles * 2 * np.pi / N
        A = (LF.translation[1] - RF.translation[1]) * 1.97 / 2

        CoP_traj = []
        CoM_traj = []
        force_traj = []
        torque_traj = []
        LF_wrench_traj = []
        RF_wrench_traj = []

        for t in time:

            tb = t - Dt
            ta = t + Dt

            Bb = np.array([0, A * np.sin(tb), 1.05])
            B = np.array([0, A * np.sin(t), 1.05])
            Ba = np.array([0, A * np.sin(ta), 1.05])

            q, dq, ddq = self.biped.solve(
                [pin.SE3(np.eye(3), Bb), pin.SE3(np.eye(3), B), pin.SE3(np.eye(3), Ba)],
                [LF, LF, LF],
                [RF, RF, RF],
                self.q0,
                Dt,
            )

            self.dyn.computeDynamics(
                q, dq, ddq, np.zeros(6), True
            )  # no distributed foreces
            self.dyn.getGroundCoMForce()
            com = pin.centerOfMass(self.dyn.model(), self.dyn.data(), q)

            self.dyn.distributeForce(
                self.dyn.getGroundCoMForce(), self.dyn.getGroundCoMTorque(), com
            )

            CoP_traj.append(self.dyn.getCoP().copy())
            CoM_traj.append(com.copy())
            force_traj.append(self.dyn.getGroundCoMForce().copy())
            torque_traj.append(self.dyn.getGroundCoMTorque().copy())
            LF_wrench_traj.append(
                self.dyn.getContact("left_sole").appliedForce().copy()
            )
            RF_wrench_traj.append(
                self.dyn.getContact("right_sole").appliedForce().copy()
            )

        CoP_xy = np.vstack(CoP_traj)
        CoM_xy = np.vstack(CoM_traj)
        force_xyz = np.vstack(force_traj)
        LF_wrench = np.vstack(LF_wrench_traj)
        RF_wrench = np.vstack(RF_wrench_traj)

        self.assertTrue((LF_wrench[:, 2] - correct_left_z_force < 1e-5).all())
        self.assertTrue((RF_wrench[:, 2] - correct_right_z_force < 1e-5).all())

        if visualize:
            figure, ax = plt.subplots(2, 1)

            ax[1].plot(time, CoP_xy[:, 1])
            ax[1].plot(time, CoM_xy[:, 1])

            ax[0].plot(time, force_xyz[:, 2])
            ax[0].plot(time, LF_wrench[:, 2])
            ax[0].plot(time, RF_wrench[:, 2])

            figure.savefig("./figures/VerticalForce_and_CoM_CoP.png", format="png")

    def test_dynamic_computation(self):
        e = 1e-12
        q0 = self.dyn.model().referenceConfigurations["half_sitting"]
        v0 = np.zeros(self.dyn.model().nv)
        self.dyn.computeDynamics(
            q0, v0, v0, np.zeros(6), True
        )  # no distributed foreces

        com_flat = self.dyn.getCoM()
        cop_flat = self.dyn.getCoP()

        self.assertTrue((com_flat[:2] - cop_flat[:2] < e).all())

        self.dyn.computeDynamics(q0, v0, v0, np.zeros(6), False)  # distributed foreces

        com_noflat = self.dyn.getCoM()
        cop_noflat = self.dyn.getCoP()

        self.assertTrue((com_noflat[:2] - cop_noflat[:2] < e).all())
        self.assertTrue((com_flat[:2] - com_noflat[:2] < e).all())
        self.assertTrue((cop_flat[:2] - cop_noflat[:2] < e).all())

        # As the acceleration is zero, the non-linearities are the same for any w
        n_1 = self.dyn.computeNL(1)
        n_9 = self.dyn.computeNL(9)

        self.assertTrue(n_1 == n_9)

        """  The following commented example corresponds to an infeasible case
              ProxSuite produces infeasible solutions in such cases, we have to
              check for infeasibility ourselves and deal with it.  """
        # self.dyn.deactivateContact6d("right_sole")
        # self.dyn.computeDynamics(q0, v0, v0, np.zeros(6), False)


if __name__ == "__main__":
    unittest.main()

    if debug:
        print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
        h = TestDynaCoM()
        h.setUp()
