#!/usr/bin/env python3
import importlib.util
import os
import unittest
import numpy as np

MODULE = os.path.join(os.path.dirname(__file__), "..", "scripts", "optimization_ik.py")
spec = importlib.util.spec_from_file_location("optimization_ik", MODULE)
opt = importlib.util.module_from_spec(spec); spec.loader.exec_module(opt)


class OptimizationIkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = os.environ["RM75_6FB_URDF"]
        cls.ik = opt.OptimizationIk(cls.urdf, max_cpu_time=.5)

    def test_symbolic_fk_matches_pinocchio_reference(self):
        q=[.2,-.5,.25,-.7,.3,.6,-.2]
        expected=np.array([.70550716378790412,-.34162561233508659,-.62092800133399195,-.40193964549762862,
          .4422264862924814,.89685799997471294,.009025669103465346,-.09388325689590678,
          .5538008456704938,-.2809584824810183,.7838156380531142,.669624072760672,0,0,0,1]).reshape(4,4)
        self.assertLess(np.max(np.abs(self.ik.forward(q)-expected)),2e-14)

    def test_ipopt_converges_and_respects_limits(self):
        target=self.ik.forward([.2,-.5,.25,-.7,.3,.6,-.2])
        result=self.ik.solve(target,[-.3,-.15,-.2,-.35,-.25,.2,.4])
        self.assertEqual(result["status"],"SUCCESS",result)
        self.assertLessEqual(result["position_error_m"],.002)
        self.assertLessEqual(result["orientation_error_rad"],np.deg2rad(2))
        self.assertGreaterEqual(result["min_limit_margin_rad"],0)

    def test_warm_start_and_invalid_input(self):
        target=self.ik.forward([.21,-.49,.26,-.69,.31,.59,-.19])
        self.assertEqual(self.ik.solve(target,[.2,-.5,.25,-.7,.3,.6,-.2])["status"],"SUCCESS")
        self.assertEqual(self.ik.solve(target,[np.nan]*7)["status"],"NUMERICAL_ERROR")

    def test_position_only_contract(self):
        seed=np.zeros(7); target=self.ik.forward(seed); target[:3,:3]=np.diag([1,-1,-1])@target[:3,:3]
        result=self.ik.solve(target,seed,orientation_enabled=False)
        self.assertEqual(result["status"],"SUCCESS",result)
        self.assertGreater(result["orientation_error_rad"],3.0)


if __name__ == "__main__": unittest.main()
