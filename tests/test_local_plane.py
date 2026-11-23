import pathlib,shutil,subprocess,tempfile,unittest


class LocalPlaneTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++') and pathlib.Path('/usr/include/eigen3/Eigen').exists(), 'Requires Linux Eigen and g++')
    def test_plane_fits_are_stable_and_reject_invalid_neighborhoods(self):
        root=pathlib.Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            exe=str(pathlib.Path(directory)/'local_plane_test')
            subprocess.run(['g++','-O1','-std=c++17','-I/usr/include/eigen3','-I'+str(root/'src/LIO-SAM/include/lio_sam'),str(root/'tests/local_plane_test.cpp'),'-o',exe],check=True,capture_output=True,timeout=90)
            subprocess.run([exe],check=True,capture_output=True,timeout=10)
