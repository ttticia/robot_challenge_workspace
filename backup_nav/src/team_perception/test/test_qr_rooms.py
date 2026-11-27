"""QR rooms: codes only appear in rooms with survivors (1-2 per room; not every room has one)."""
import pytest

from team_perception.poster_utils import POSTER, PosterTracker, QrRooms


def test_room_link_is_by_distance_not_cell():
    q = QrRooms(radius=1.0)
    q.add('A', (2.0, 0.0, 0.3), 0.0)
    assert [r.text for r in q.near((2.8, 0.4, 0.3))] == ['A']      # same room, other cell
    assert q.near((3.5, 0.0, 0.3)) == []
    q.add('A', (2.2, 0.0, 0.3), 1.0)                                # repeat read refines it
    assert q.rooms['A'].room[0] == pytest.approx(2.1) and q.rooms['A'].n == 2


def test_fewer_votes_needed_near_a_code():
    t = PosterTracker(0.5, need=5)
    for k in range(2):
        tr, rep = t.update((1.0, 0.0, 0.0), POSTER, float(k), need=3)
        assert not rep
    tr, rep = t.update((1.0, 0.0, 0.0), POSTER, 2.0, need=3)
    assert rep and t.is_poster(tr)
    other = PosterTracker(0.5, need=5)
    for k in range(3):
        o, rep = other.update((1.0, 0.0, 0.0), POSTER, float(k))
    assert not rep                                                  # default still 5


def test_look_request_after_delay_and_only_once():
    q = QrRooms()
    q.add('A', (2.0, 0.0, 0.0), 10.0)
    assert q.to_look(12.0, 3.0) == []
    (r,) = q.to_look(13.5, 3.0)
    r.look_requested = True
    assert q.to_look(20.0, 3.0) == []
    q.add('B', (5.0, 0.0, 0.0), 10.0)
    q.poster_confirmed((5.3, 0.0, 0.0))                             # poster already found
    assert q.to_look(20.0, 3.0) == []


def test_backup_when_robot_leaves_or_timeout():
    q = QrRooms()
    q.add('A', (2.0, 0.0, 0.0), 0.0)
    assert q.to_report((4.0, 0.0, 0.0), 1.0, 1.5, 90.0) == []       # read from afar: wait
    assert q.to_report((2.5, 0.0, 0.0), 5.0, 1.5, 90.0) == []       # now in the room
    assert [r.text for r in q.to_report((4.0, 0.0, 0.0), 6.0, 1.5, 90.0)] == ['A']   # left
    assert [r.text for r in q.to_report(None, 91.0, 1.5, 90.0)] == ['A']   # no TF: timeout
    q.rooms['A'].backup = True
    assert q.to_report((9.0, 0.0, 0.0), 100.0, 1.5, 90.0) == []     # reported once


def test_no_backup_if_a_poster_was_confirmed_there():
    q = QrRooms()
    q.add('A', (2.0, 0.0, 0.0), 0.0)
    assert q.poster_confirmed((2.4, 0.3, 0.0)) is None              # no backup to claim
    assert q.to_report((9.0, 0.0, 0.0), 100.0, 1.5, 90.0) == []


def test_first_poster_after_backup_is_that_survivor_second_is_new():
    q = QrRooms()
    q.add('A', (2.0, 0.0, 0.0), 0.0)
    q.rooms['A'].backup = True                                     # reported from the QR
    first = q.poster_confirmed((2.3, 0.2, 0.0))
    assert first is not None and first.text == 'A'                 # no new "Human found"
    assert q.poster_confirmed((1.8, -0.4, 0.0)) is None            # 2nd survivor: announce
