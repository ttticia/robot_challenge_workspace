import math
from types import SimpleNamespace
import numpy as np
from lite3_maze.navigation_core import GridMap, ScanOdometry, FrontierPlanner, astar, rotation
from lite3_maze.cells import CellExplorer
from lite3_maze.core import Grid
from lite3_maze.geometry import cloud_xyz
from lite3_maze.geometry import ground_height


def test_ground_height_with_sparse_floor_and_dense_walls():
    floor = np.tile([.5, .2, -.35], (30, 1))
    walls = np.tile([1., .5, .2], (1000, 1))
    assert np.isclose(ground_height(np.vstack((floor, walls))), -.35)
    assert ground_height(walls) is None
    assert ground_height(np.vstack((floor[:19], walls))) is None
    distant_floor = floor.copy()
    distant_floor[:, 0] = 3.
    assert ground_height(np.vstack((distant_floor, walls))) is None


def test_scan_translation_and_unobservable_wall():
    rng=np.random.default_rng(1)
    world=rng.uniform([-1.,-1.],[2.,1.5],(300,2))
    odom=ScanOdometry();odom.update(world,0.)
    shift=np.array([.025,-.012]);yaw=.04
    xy,heading=odom.update((world-shift)@rotation(yaw),yaw)
    assert odom.valid and np.linalg.norm(xy-shift)<.005
    assert abs(heading-yaw)<1e-6
    wall=np.column_stack((np.linspace(-2.,2.,100),np.ones(100)))
    odom=ScanOdometry();odom.update(wall,0.);odom.update(wall-[.02,0.],0.)
    assert not odom.valid


def test_unknown_and_corners_block_motion():
    grid=GridMap(resolution=.02,extent=2.)
    assert not grid.footprint_clear([0.,0.],0.)
    grid.log[:]=-4
    assert grid.footprint_clear([0.,0.],0.)
    grid.mark_obstacles(np.array([[.31,.22]]))
    assert not grid.footprint_clear([0.,0.],0.)
    free=np.ones((20,20),bool);free[:,10]=False
    assert not astar(free,(3,3),(16,16))
    assert not astar(free,(-1,0),(4,4))
    assert not astar(free,(4,4),(25,0))
    free[8:12,10]=True
    assert astar(free,(3,3),(16,16))


def test_existing_corner_overlap_can_recenter():
    # Captured rough-maze failure: rotating retains a current corner overlap.
    g=GridMap(resolution=.04,extent=4.)
    g.log[:]=-4
    g.mark_obstacles(np.array([[.46,1.50]]))
    xy=np.array([.369,1.273]);yaw=2.66
    p=FrontierPlanner(g);p.goal=np.array([.02,1.82])
    p.path=[xy,np.array([.30,1.35]),np.array([.22,1.54]),p.goal]
    before=g.footprint_obstructions(xy,yaw)
    assert before
    cmd,state=p.velocity(xy,yaw)
    assert np.linalg.norm(cmd)>0,(cmd,state)
    for t in np.linspace(.05,.35,7):
        after=g.footprint_obstructions(xy+rotation(yaw+cmd[2]*t/2)@cmd[:2]*t,yaw+cmd[2]*t)
        assert after.issubset(before)
        before=after
    assert not before


def test_open_start_and_unknown_cannot_prove_exit():
    g=Grid(np.zeros((100,100),int),.05,(-2.5,-2.5))
    c=CellExplorer(minimum_travel=.1,exit_hold=.1)
    for i in range(20):c.observe(g,(i*.05,0.,0.),i*.1)
    assert not c.complete  # Never saw an enclosed start.
    c.enclosed_start=True;c.visits.update({(2,0):1,(3,0):1})
    unknown=Grid(np.full((100,100),-1),.05,(-2.5,-2.5))
    for i in range(5):c.observe(unknown,(1.,0.,0.),3+i)
    assert not c.complete
    for i in range(5):c.observe(g,(1.,0.,0.),10+i)
    assert c.complete


def test_livox_xyz_extra_fields_endian_and_padded_rows():
    for endian in ('<','>'):
        fields=[SimpleNamespace(name=n,offset=o,datatype=7,count=1) for n,o in [('x',0),('y',4),('z',8),('intensity',12)]]
        data=bytearray(80)
        arr=np.ndarray((2,2,4),dtype=endian+'f4',buffer=data,strides=(40,16,4))
        arr[:]=[[[1,2,3,9],[4,5,6,9]],[[7,8,9,9],[float('nan'),1,2,9]]]
        msg=SimpleNamespace(height=2,width=2,row_step=40,point_step=16,fields=fields,data=data,is_bigendian=endian=='>')
        assert np.array_equal(cloud_xyz(msg),[[1,2,3],[4,5,6],[7,8,9]])


def test_frontier_goal_preserves_room_to_rotate():
    g=GridMap(resolution=.04,extent=5.)
    ij=np.indices(g.log.shape).reshape(2,-1).T[:,::-1]
    points=g.world(ij)
    free=(points[:,0]>-.48)&(points[:,0]<3.5)&(abs(points[:,1])<.48)
    wall=((abs(points[:,1])>=.48)&(abs(points[:,1])<.55)&(points[:,0]<3.5))
    wall|=(points[:,0]<=-.48)&(points[:,0]>-.55)&(abs(points[:,1])<.55)
    g.log[ij[free,1],ij[free,0]]=-4
    g.log[ij[wall,1],ij[wall,0]]=4
    p=FrontierPlanner(g)
    xy=np.array([2.5,.02])
    p.plan(xy,yaw=0.)
    assert p.goal is not None
    assert g.planning_free()[tuple(g.cell(p.goal)[::-1])]
    for heading in np.linspace(-math.pi,math.pi,25):
        assert g.footprint_clear(p.goal,heading),'Frontier must allow the next turn'


def test_delayed_scan_recovers_without_accepting_an_impossible_fast_jump():
    rng=np.random.default_rng(51)
    world=rng.uniform([-2.,-1.],[2.,2.],(500,2))
    o=ScanOdometry();o.update(world,0.,0.)
    # 11 cm between 0.3-second capture stamps is feasible after a dropped scan.
    xy,_=o.update(world-[.11,0.],0.,.3)
    assert o.valid and abs(xy[0]-.11)<.01
    # The same displacement over 10 ms is inconsistent with this robot speed.
    o=ScanOdometry();o.update(world,0.,0.)
    o.update(world-[.11,0.],0.,.01)
    assert not o.valid
