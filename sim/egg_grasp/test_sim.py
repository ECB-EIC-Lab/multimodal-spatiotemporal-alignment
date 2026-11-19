import unittest
import numpy as np
from scipy.optimize import minimize
from contact import WEIGHT, project, deformation_fit, simulate
from motion import trajectory, check_path
from run import wilson
from vision import camera_study
from timing import timing_study

class OfflineChecks(unittest.TestCase):
    def test_projection_reference(self):
        rng=np.random.default_rng(410)
        for _ in range(35):
            lo=rng.uniform(-1,0,3)
            hi=rng.uniform(.05,1.,3)
            g=rng.normal(size=3)
            z=rng.normal(size=3)
            b=float(np.minimum(lo*g,hi*g).sum()+rng.uniform(0,.9))
            x,box,plane=project(z[None,:],lo[None,:],hi[None,:],g,np.array([b]))
            ref=minimize(lambda r: .5*((r-z)**2@WEIGHT), np.where(g>0,lo,hi),
                         method='SLSQP',jac=lambda r: WEIGHT*(r-z),bounds=list(zip(lo,hi)),
                         constraints=[dict(type='ineq',fun=lambda r:b-g@r,jac=lambda r:-g)],
                         options={'ftol':1e-12,'maxiter':100})
            self.assertTrue(ref.success and box[0] and plane[0])
            np.testing.assert_allclose(x[0],ref.x,atol=3e-6)
            self.assertLessEqual(g@x[0],b+1e-8)

    def test_infeasibility_and_boundary(self):
        z=np.ones((1,3))
        lo=np.array([[.2,.2,.2]])
        hi=lo+.5
        g=np.array([1.,0.,-1.])
        minimum=np.min([lo*g,hi*g],axis=0).sum(axis=1)
        x,box,plane=project(z,lo,hi,g,minimum)
        self.assertTrue(box[0] and plane[0])
        self.assertAlmostEqual(float(x[0]@g),minimum[0])
        self.assertFalse(project(z,lo,hi,g,minimum-.001)[2][0])
        self.assertFalse(project(z,hi,lo)[1][0])
        self.assertTrue(project(z,lo,hi,np.zeros(3),np.zeros(1))[2][0])

    def test_task_endpoints_and_ik(self):
        for mode in ('gentle','brisk'):
            p=trajectory(mode)
            self.assertEqual(p[-1][0],'supported')
            np.testing.assert_allclose(p[-1][1],[0,.1,0])
            self.assertAlmostEqual(max(x[1][2] for x in p),.05)
            rows=check_path(mode)
            self.assertGreater(min(x['limit_margin_rad'] for x in rows),0)
            self.assertLess(max(x['pos_error_mm'] for x in rows),.01)
            self.assertLess(max(x['angle_error_deg'] for x in rows),.01)

    def test_common_abort_accounting(self):
        fit=deformation_fit()
        for m in ('A','B','B_noD','shear'):
            x,_=simulate('peeled','gentle',m,55,30,fit,stress='stale_view')
            self.assertFalse((x['outcome']=='complete').any())
            self.assertTrue((x['steps']<=10).all())
            np.testing.assert_array_equal(x['steps'],x['coverage'][:,0])
        a,_=simulate('raw','brisk','B',110,12,fit)
        b,_=simulate('raw','brisk','B_noD',110,12,fit)
        np.testing.assert_array_equal(a['outcome'],b['outcome'])
        np.testing.assert_array_equal(a['peak_N'],b['peak_N'])

    def test_shared_edge_semantics(self):
        arrival,shared,delayed=timing_study()
        self.assertLess(shared['abs_p95_ms'],arrival['abs_p95_ms'])
        self.assertAlmostEqual(delayed['mean_bias_ms']-shared['mean_bias_ms'],6.)

    def test_wilson_and_camera(self):
        self.assertTrue(wilson(0,100)[1]<.04)
        self.assertTrue(wilson(100,100)[0]>.96)
        vision=camera_study()
        self.assertLess(vision[1]['mae_fraction'],vision[0]['mae_fraction'])
        self.assertLess(vision[2]['valid'],vision[2]['n'])

if __name__=='__main__':
    unittest.main()
