import math
import cv2
import numpy as np
import pytest
from lite3_mission.core import Grid,EventStore,VelocityLimiter,relative_goal,select_frontier,transform_point
from lite3_mission.vision import bgr_image,qr_position,parse_client_output


def test_rotated_map_roundtrip():
    grid=Grid(np.zeros((100,100)),.04,(1.,-2.),math.pi/2)
    cells=np.array([[4,8],[70,40]])
    assert np.array_equal(grid.cell(grid.world(cells)),cells)
    assert relative_goal((1.,2.,math.pi/2),(.5,0.,0.))==pytest.approx((1.,2.5,math.pi/2))


def test_frontier_is_known_reachable_and_wall_does_not_create_frontiers():
    data=np.full((160,180),-1,dtype=int)
    data[10:150,10:100]=0
    data[10:150,9]=100
    data[9,9:101]=100
    data[150,9:101]=100
    grid=Grid(data,.04,(0.,0.))
    point,reason=select_frontier(grid,grid.world([40,80]),clearance=.2)
    assert reason=='FRONTIER' and point is not None
    cell=grid.cell(point[:2])
    assert grid.data[cell[1],cell[0]]==0 and cell[0]<100
    # Close the end of the corridor. Unseen space beyond the wall is not a target.
    data[9:151,100]=100
    point,reason=select_frontier(Grid(data,.04,(0.,0.)),grid.world([40,80]),clearance=.2)
    assert point is None and reason=='NO_REACHABLE_FRONTIER'
    assert select_frontier(grid,grid.world([0,0]))[1]=='START_NOT_CLEAR'


def test_velocity_norm_ramp_and_immediate_stop():
    limiter=VelocityLimiter(linear=.1,angular=.25,acceleration=.2,angular_acceleration=.5)
    first=limiter.apply([1.,1.,2.],.05,True)
    assert np.linalg.norm(first[:2])==pytest.approx(.01)
    for _ in range(40):result=limiter.apply([1.,1.,2.],.05,True)
    assert np.linalg.norm(result[:2])==pytest.approx(.1)
    assert result[2]==pytest.approx(.25)
    assert not limiter.apply([1,0,0],.05,False).any()
    assert not limiter.apply([float('nan'),0,0],.05,True).any()
    assert not limiter.apply([0,0,0],.05,True).any()


def test_transform_dedup_and_calibrated_cells():
    q=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)]
    position=transform_point([1.,0.,0.],[2.,3.,.5],q)
    assert position==pytest.approx([2.,4.,.5])
    store=EventStore()
    event=dict(kind='survivor',label='person',position=position,confidence=.8,stamp={'sec':1,'nanosec':0})
    assert store.add(event)[1]
    assert not store.add(dict(event,position=[2.02,4.,.5]))[1]
    assert len(store.records)==1 and store.records[0]['observations']==2
    assert store.records[0]['cell'] is None
    calibrated=EventStore(cell_origin=[0.,0.])
    record,_=calibrated.add(event)
    assert record['cell']==[2,5]
    with pytest.raises(ValueError):store.add(dict(event,position=[float('nan'),0,0]))
    with pytest.raises(ValueError):EventStore(cell_size=0.)
    with pytest.raises(ValueError):EventStore(cell_origin=[float('inf'),0.])


def test_image_row_padding_and_qr_pose():
    rows=np.array([[255,0,0,0,255,0,99,99],[0,0,255,255,255,255,99,99]],dtype=np.uint8)
    bgr=bgr_image(rows.tobytes(),2,2,8,'rgb8')
    assert bgr[0,0].tolist()==[0,0,255]
    size=.12
    obj=np.array([[-.06,.06,0],[.06,.06,0],[.06,-.06,0],[-.06,-.06,0]],float)
    matrix=np.array([[600.,0,320],[0,600.,240],[0,0,1.]])
    truth=np.array([.12,-.05,1.2])
    corners,_=cv2.projectPoints(obj,np.array([math.pi,.1,.02]),truth,matrix,None)
    error,position=qr_position(corners,matrix,[],size)
    assert error<.01 and position==pytest.approx(truth,abs=.002)
    with pytest.raises(ValueError):qr_position(corners,matrix,[],0.)


def test_scene_parser_requires_structured_result_and_never_returns_actions():
    import json
    inner=dict(room_type='kitchen',survivor_present=True,confidence=.9,evidence=['sink'],robot_actions=['run'])
    text='HTTP 200 OK\nClient elapsed: 2.0 s\n'+json.dumps(dict(success=True,result=json.dumps(inner)))
    result=parse_client_output(text)
    assert result['room_type']=='kitchen' and 'robot_actions' not in result
    with pytest.raises(ValueError):parse_client_output('{"success":false,"error":"busy"}')
    inner['confidence']=True
    with pytest.raises(ValueError):parse_client_output(json.dumps(dict(success=True,result=inner)))


def test_room_profile_keeps_2d_obstacles_and_does_not_drop_independent_3d_sources():
    from copy import deepcopy
    from lite3_mission.room_config import room_parameters
    scan=dict(topic='/scan',data_type='LaserScan',marking=True,clearing=True)
    cm=dict(plugins=['static_layer','obstacle_layer','voxel_layer','inflation_layer'],
            footprint='measured-footprint',
            obstacle_layer=dict(enabled=True,observation_sources='scan',scan=scan),
            voxel_layer=dict(observation_sources='scan',scan=deepcopy(scan)))
    original=dict(controller_server=dict(ros__parameters=dict(FollowPath=dict(plugin='nav2_mppi_controller::MPPIController'))),
                  planner_server=dict(ros__parameters=dict(GridBased={})),
                  local_costmap=dict(local_costmap=dict(ros__parameters=cm)))
    result=room_parameters(original,'trial.xml')['local_costmap']['local_costmap']['ros__parameters']
    assert result['plugins']==['static_layer','obstacle_layer','inflation_layer']
    assert result['footprint']==cm['footprint']
    assert result['obstacle_layer']['scan']['marking'] and result['obstacle_layer']['scan']['clearing']
    cm['voxel_layer']['observation_sources']='depth'
    result=room_parameters(original,'trial.xml')['local_costmap']['local_costmap']['ros__parameters']
    assert 'voxel_layer' in result['plugins']
