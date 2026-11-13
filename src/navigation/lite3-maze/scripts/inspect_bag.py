#!/usr/bin/env python3
"""Read the supplied SQLite bag without replaying commands or modifying it."""
import argparse
import json
import sqlite3
import math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from lite3_maze.geometry import cloud_xyz


def inspect(path):
    db=sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)
    first,last,count=db.execute('SELECT min(timestamp),max(timestamp),count(*) FROM messages').fetchone()
    result=dict(path=str(path),duration_seconds=(last-first)*1e-9,message_count=count,topics=[])
    for tid,name,kind in db.execute('SELECT id,name,type FROM topics'):
        total=db.execute('SELECT count(*) FROM messages WHERE topic_id=?',(tid,)).fetchone()[0]
        entry=dict(topic=name,type=kind,messages=total)
        raw=db.execute('SELECT data FROM messages WHERE topic_id=? LIMIT 1',(tid,)).fetchone()
        if raw:
            try:
                msg=deserialize_message(raw[0],get_message(kind))
                if hasattr(msg,'header'):
                    entry.update(frame=msg.header.frame_id,first_header_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
                if name=='/tf_static':
                    entry['transforms']=[dict(parent=t.header.frame_id,child=t.child_frame_id,
                        translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]) for t in msg.transforms]
                if hasattr(msg,'fields'):
                    entry['point_fields']=[dict(name=f.name,offset=f.offset,type=f.datatype) for f in msg.fields]
            except (ModuleNotFoundError,ImportError):
                entry['decoder']='custom ROS interface unavailable locally'
        result['topics'].append(entry)
    point_type=get_message('sensor_msgs/msg/PointCloud2')
    decoded,valid_points=0,0
    for raw, in db.execute("SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/livox/points')"):
        cloud=deserialize_message(raw,point_type)
        valid_points+=len(cloud_xyz(cloud))
        decoded+=1
    result['point_cloud_validation']=dict(decoded_frames=decoded,finite_xyz_points=valid_points,
        accepts_extra_livox_fields=True)
    db.close()
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path)
    p.add_argument('--output',type=Path,default=Path('docs/bag_inspection.json'))
    a=p.parse_args();result=inspect(a.bag)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
