"""PointCloud2 layout decoding with no assumptions about Livox extra fields."""
import numpy as np


def ground_height(level):
    """Estimate floor height from nearby points below the standing robot body."""
    nearby = level[np.linalg.norm(level[:, :2], axis=1) < 2.]
    ground = nearby[(nearby[:, 2] > -.65) & (nearby[:, 2] < -.12)]
    if len(ground) < 20:
        return None
    return float(np.quantile(ground[:, 2], .1))


def cloud_xyz(msg):
    if msg.height < 1 or msg.width < 1 or msg.row_step < msg.width*msg.point_step:
        return np.empty((0,3))
    fields = {f.name:f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in ('x','y','z')):
        raise ValueError('XYZ must be scalar FLOAT32 fields')
    dtype = np.dtype(dict(names=['x','y','z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
        offsets=[fields[n].offset for n in ('x','y','z')], itemsize=msg.point_step))
    points = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                        strides=(msg.row_step,msg.point_step))
    xyz = np.column_stack([points[n].ravel() for n in ('x','y','z')]).astype(float)
    return xyz[np.isfinite(xyz).all(axis=1)]
