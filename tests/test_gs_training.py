"""CPU geometry/loss checks; use the optional 3DGS Python environment."""
import importlib.util
import pathlib
import tempfile
import unittest

import numpy as np

AVAILABLE = all(importlib.util.find_spec(k) is not None for k in ['torch', 'gsplat'])
if AVAILABLE:
    import torch
    from refine_local_gs import pose_delta, ssim_loss, CappedStrategy, append_parameters
    from train_local_gs import export_gaussians
    from clean_local_gs import cleanup_masks


@unittest.skipUnless(AVAILABLE, 'Requires optional torch/gsplat environment')
class GaussianTrainingTests(unittest.TestCase):
    def test_cleanup_removes_traced_floaters_but_keeps_nearby_static_surface(self):
        # Actual dirty-pixel contributor, unsupported sky splat, and nearby static surface.
        modes,_=cleanup_masks(np.array([6,0,10]),np.array([2,0,0]),np.array([4,0,8]),np.array([2,0,0]),
                              np.array([.12,5.67,.5]),np.array([.092,1.57,.02]),np.array([.028,.325,.05]),np.array([.903,.953,.9]))
        self.assertEqual(modes['balanced'].tolist(),[False,False,True])
        self.assertEqual(modes['original'].tolist(),[True,True,True])

    def test_ssim_prefers_aligned_edges_and_has_descent_gradient(self):
        image = torch.zeros(24, 32, 3); image[:, 16:] = .8
        blurred = torch.nn.functional.avg_pool2d(image.permute(2,0,1)[None],5,1,2)[0].permute(1,2,0).detach().requires_grad_()
        self.assertAlmostEqual(ssim_loss(image,image).item(), 0, places=5)
        initial = ssim_loss(blurred,image); initial.backward()
        with torch.no_grad(): improved = ssim_loss(blurred-.0001*blurred.grad.sign(),image)
        self.assertTrue(torch.isfinite(blurred.grad).all())
        self.assertLess(improved.item(),initial.item())

    def test_pose_is_rigid_with_finite_gradient_at_identity(self):
        v=torch.zeros(2,6,requires_grad=True);m=pose_delta(v)
        np.testing.assert_allclose(m.detach(),np.tile(np.eye(4),(2,1,1)),atol=1e-7)
        (m[:,0,1].sum()+m[:,2,3].sum()).backward()
        self.assertTrue(torch.isfinite(v.grad).all())
        moved=pose_delta(torch.tensor([[.1,-.2,.3,0.,0.,.1]]))
        rotation=moved[0,:3,:3].detach().numpy()
        np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-6)
        self.assertAlmostEqual(np.linalg.det(rotation),1.,places=6)

    def test_masked_ssim_does_not_optimize_excluded_pixels(self):
        target=torch.rand(24,32,3);prediction=target.clone();prediction[:,:12]=0
        prediction.requires_grad_();mask=torch.ones(24,32);mask[:,:12]=0
        loss=ssim_loss(prediction,target,mask);loss.backward()
        self.assertAlmostEqual(loss.item(),0.,places=6)
        self.assertEqual(torch.count_nonzero(prediction.grad[:,:12]).item(),0)

    def test_growth_budget_survives_split_and_optimizer_replacement(self):
        params=torch.nn.ParameterDict({
            'means':torch.nn.Parameter(torch.zeros(2,3)),
            'quats':torch.nn.Parameter(torch.tensor([[1.,0,0,0],[1.,0,0,0]])),
            'scales':torch.nn.Parameter(torch.full((2,3),-2.)),
            'opacities':torch.nn.Parameter(torch.zeros(2))})
        opts={k:torch.optim.Adam([p]) for k,p in params.items()}
        strategy=CappedStrategy();strategy.point_cap=3
        state=strategy.initialize_state();state['grad2d']=torch.ones(2);state['count']=torch.ones(2)
        strategy._grow_gs(params,opts,state,1000)
        self.assertEqual(len(params['means']),3)
        for k,p in params.items():self.assertIs(opts[k].param_groups[0]['params'][0],p)
        state['grad2d'].fill_(1);state['count'].fill_(1)
        strategy._grow_gs(params,opts,state,1100)
        self.assertEqual(len(params['means']),3)

    def test_feature_append_keeps_existing_adam_moments(self):
        params=torch.nn.ParameterDict({'means':torch.nn.Parameter(torch.ones(2,3))})
        optimizer=torch.optim.Adam([params['means']]);params['means'].square().sum().backward();optimizer.step()
        before=optimizer.state[params['means']]['exp_avg'].clone()
        state={'grad2d':torch.ones(2),'count':torch.ones(2),'scene_scale':3.}
        append_parameters(params,{'means':optimizer},state,{'means':torch.zeros(1,3)})
        torch.testing.assert_close(optimizer.state[params['means']]['exp_avg'][:2],before)
        self.assertEqual(optimizer.state[params['means']]['exp_avg'][2].abs().sum().item(),0)
        self.assertEqual(len(state['count']),3)
        self.assertEqual(state['scene_scale'],3.)

    def test_export_preserves_sh3_channels_and_log_scale(self):
        coeff=torch.arange(48,dtype=torch.float32).reshape(1,16,3)
        with tempfile.TemporaryDirectory() as folder:
            path=pathlib.Path(folder)/'model.ply'
            export_gaussians(path,torch.tensor([[1.,2.,3.]]),torch.tensor([[2.,0.,0.,0.]]),torch.tensor([[-2.,-3.,-4.]]),torch.tensor([.2]),coeff)
            with path.open('rb') as f:
                names=[]
                while True:
                    line=f.readline().decode().strip()
                    if line.startswith('property float '):names.append(line.split()[-1])
                    if line=='end_header':break
                data=np.frombuffer(f.read(),dtype=[(k,'<f4') for k in names])
            self.assertEqual(len(data),1)
            np.testing.assert_allclose([data[f'f_rest_{j}'][0] for j in range(45)],coeff[0,1:].numpy().T.ravel())
            self.assertEqual(data['rot_0'][0],1.)
            self.assertEqual(data['scale_1'][0],-3.)


if __name__=='__main__':unittest.main()
