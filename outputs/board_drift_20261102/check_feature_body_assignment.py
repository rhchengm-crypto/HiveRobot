"""Offline sensitivity check only; never writes production configuration."""
from pathlib import Path

folder = Path(__file__).resolve().parent
source = (folder / 'fit_approach_camera.py').read_text()
source = source.replace(
    "pixels=np.array([f['points'] for f in records]);valid=np.array([f['valid'] for f in records],bool);n=pixels.shape[1]",
    "pixels=np.array([f['points'] for f in records])[:,:5];valid=np.array([f['valid'] for f in records],bool)[:,:5];n=pixels.shape[1]",
)
source = source.replace('approach_camera_fit.json', 'approach_camera_fit_without_feature5.json')
source = source.replace('approach_camera_fit.npz', 'approach_camera_fit_without_feature5.npz')
exec(compile(source, str(folder / 'fit_approach_camera.py'), 'exec'))
