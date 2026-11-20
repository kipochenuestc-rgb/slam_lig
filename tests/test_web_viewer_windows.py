import json,os,pathlib,shutil,socket,subprocess,time,unittest,urllib.request

ROOT=pathlib.Path(__file__).resolve().parents[1]

@unittest.skipUnless(os.name=='nt','Windows viewer launcher tests')
class WebViewerTests(unittest.TestCase):
    def test_launcher_finds_bundled_node_without_node_on_path(self):
        environment={**os.environ,'PATH':os.environ['SystemRoot']+'\\System32;'+os.environ['SystemRoot']+'\\System32\\WindowsPowerShell\\v1.0'}
        result=subprocess.run(['cmd.exe','/d','/c',str(ROOT/'web_viewer/start.cmd'),'--check'],capture_output=True,text=True,encoding='utf-8',env=environment,timeout=45)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        health=json.loads(result.stdout.strip());self.assertEqual(health['service'],'slam_lig_web_viewer')

    def test_static_map_and_private_config_api_and_duplicate_start(self):
        node=shutil.which('node')
        if not node:self.skipTest('Node not installed')
        with socket.socket() as available:available.bind(('127.0.0.1',0));port=available.getsockname()[1]
        command=[node,str(ROOT/'web_viewer/server.mjs'),'--port',str(port)]
        process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        base=f'http://127.0.0.1:{port}'
        try:
            deadline=time.monotonic()+10
            while True:
                try:
                    with urllib.request.urlopen(base+'/api/health',timeout=1) as response:health=json.load(response)
                    break
                except OSError:
                    if time.monotonic()>deadline:raise
                    time.sleep(.1)
            self.assertEqual(health['service'],'slam_lig_web_viewer')
            with urllib.request.urlopen(base+'/api/amap-config') as response:raw=response.read().decode();config=json.loads(raw)
            self.assertNotIn('securityJsCode',config);self.assertNotIn('code',config)
            actual=ROOT/'web_viewer/config/amap.local.json'
            if actual.exists():
                secret=json.loads(actual.read_text(encoding='utf-8'))['securityJsCode']
                if secret:self.assertTrue(secret not in raw)
            self.assertTrue(config['serviceHost'].endswith(f':{port}/_AMapService'))
            if health['dataReady']:
                with urllib.request.urlopen(base+'/data/metadata.json') as response:metadata=json.load(response)
                with urllib.request.urlopen(base+'/data/points.bin') as response:points=response.read()
                self.assertEqual(len(points),metadata['pointCount']*16)
            duplicate=subprocess.run(command,capture_output=True,text=True,timeout=10)
            self.assertEqual(duplicate.returncode,0);self.assertIn('Already running',duplicate.stdout)
        finally:
            process.terminate();process.wait(timeout=5)
