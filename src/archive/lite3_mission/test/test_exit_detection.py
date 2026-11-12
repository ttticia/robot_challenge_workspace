import numpy as np
import pytest
from lite3_mission.core import Grid,select_frontier
from lite3_mission.exit_detection import ExitDetector,observed_clearance


def scene():
    a=np.full((160,200),-1,dtype=int)
    a[65:95,5:95]=0    # Narrow start corridor, connected to a large open end.
    a[10:150,90:190]=0
    g=Grid(a,.05,(0.,0.));return g,observed_clearance(g)


def test_exit_requires_enclosed_start_displacement_and_observed_open_space():
    g,c=scene();d=ExitDetector(dwell_seconds=1.)
    assert not d.update(g,c,g.world([30,80]),0.)
    for now in np.arange(.2,1.5,.2):found=d.update(g,c,g.world([140,80]),float(now))
    assert found and d.seen_enclosed
    # Spawn in an open room: do not call the starting room an exit.
    d=ExitDetector(dwell_seconds=.5)
    for now in np.arange(0,3,.2):assert not d.update(g,c,g.world([140,80]),float(now))


def test_unknown_space_map_edges_and_interrupted_dwell_are_not_an_exit():
    g,c=scene();d=ExitDetector(dwell_seconds=1.)
    d.update(g,c,g.world([30,80]),0.)
    assert not d.update(g,c,g.world([140,80]),.2)
    assert not d.update(g,c,g.world([140,80]),2.)  # Data gap resets dwell.
    assert not d.update(g,c,g.world([100,80]),2.2) # Too close to unknown boundary.
    for t in np.arange(2.4,3.4,.2):assert not d.update(g,c,g.world([0,0]),float(t))
    edge=Grid(np.zeros((100,100),int),.05,(0.,0.))
    assert observed_clearance(edge)[0,0]==pytest.approx(.05)


def test_frontier_can_seed_from_nearby_connected_free_cells_at_startup():
    a=np.full((100,120),-1,int);a[10:90,10:100]=0
    g=Grid(a,.05,(0.,0.));p,reason=select_frontier(g,g.world([11,50]),clearance=.25)
    assert p is not None and reason=='FRONTIER'
    a[10:90,14]=100  # Do not bridge the wall to reach the safe component.
    assert select_frontier(Grid(a,.05,(0.,0.)),g.world([11,50]),clearance=.25)[0] is None
