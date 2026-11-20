"""Exercise the real cmd.exe entrypoint without starting mapping or recording."""
import json,os,pathlib,subprocess,unittest

@unittest.skipUnless(os.name=='nt','Windows cmd.exe integration test')
class WindowsMenuTests(unittest.TestCase):
    def launch(self,answer):
        root=pathlib.Path(__file__).resolve().parents[1]
        result=subprocess.run(['cmd.exe','/d','/c',str(root/'start_mapping.cmd'),'--dry-run'],input=answer,capture_output=True,encoding='utf-8',timeout=60)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertNotIn('not recognized',result.stdout+result.stderr)
        marker='COMMAND_JSON='
        return json.loads(result.stdout.split(marker,1)[1].splitlines()[0]) if marker in result.stdout else None

    def test_live(self):
        command=self.launch('1\n');self.assertEqual(command[-2:],['--mode','live'])

    def test_record(self):
        command=self.launch('2\n');self.assertEqual(command[-2:],['--mode','record'])

    def test_replay_default(self):
        command=self.launch('3\n\n');self.assertEqual(command[-2:],['--mode','replay'])

    def test_replay_path_with_spaces_and_chinese(self):
        path='D:\\new data\\采集任务\\bag';command=self.launch('3\n"'+path+'"\n')
        self.assertEqual(command[-4:],['--mode','replay','--bag',path])

    def test_invalid_selection_does_not_start_replay(self):
        self.assertIsNone(self.launch('wrong\n0\n'))

    def test_closed_input_cancels(self):
        self.assertIsNone(self.launch(''))
