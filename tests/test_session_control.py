import os,pathlib,subprocess,tempfile,unittest
from unittest.mock import patch
import session_control as sessions

class SessionOwnershipTests(unittest.TestCase):
    def test_mapper_requires_exact_executable_and_project_config(self):
        config=str(sessions.ROOT/'config/params.yaml')
        self.assertTrue(sessions.is_mapper({'command':['/ws/lib/lio_sam_mapOptimization','--params-file',config]}))
        self.assertFalse(sessions.is_mapper({'command':['bash','-c','lio_sam_mapOptimization '+config]}))
        self.assertFalse(sessions.is_mapper({'command':['/ws/lib/lio_sam_mapOptimization','--params-file','/other/params.yaml']}))

    def test_pid_reuse_is_not_signalled(self):
        old={'pid':123,'start_ticks':'10','command':['sleep','10']}
        with patch.object(sessions,'process_info',return_value={**old,'start_ticks':'11'}),patch.object(sessions.os,'kill') as kill:
            sessions.send(old,2);kill.assert_not_called()

    def test_run_ownership_is_limited_to_project_runs(self):
        self.assertEqual(sessions.owned_run(str(sessions.ROOT/'runs/test')),sessions.ROOT/'runs/test')
        self.assertIsNone(sessions.owned_run('/tmp/other_project/run'))
        self.assertIsNone(sessions.owned_run(str(sessions.ROOT/'runs')))

    @unittest.skipUnless(os.name=='posix','Linux process isolation test')
    def test_orphan_cleanup_preserves_unrelated_process(self):
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory);run=root/'runs/orphan';run.mkdir(parents=True)
            owned=subprocess.Popen(['sleep','60'],env={**os.environ,'SLAM_RUN_DIR':str(run)})
            unrelated=subprocess.Popen(['sleep','60'],env={k:v for k,v in os.environ.items() if k!='SLAM_RUN_DIR'})
            try:
                with patch.object(sessions,'ROOT',root):result=sessions.stop_all(False)
                owned.wait(timeout=3);self.assertIsNone(unrelated.poll());self.assertIn(str(run),result['stopped_runs'])
            finally:
                for process in [owned,unrelated]:
                    if process.poll() is None:process.terminate()
                    process.wait(timeout=3)
