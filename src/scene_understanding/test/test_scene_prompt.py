from scene_understanding.image_folder_publisher import is_junction_file
from scene_understanding.scene_prompt import (
    build_prompt, parse_scene_json)


def test_junction_file_names():
    assert is_junction_file('junction01.jpg')
    assert is_junction_file('Junction01.jpg')        # case-insensitive
    assert is_junction_file('/a/b/JUNCTION05.png')
    assert is_junction_file('junction03_extra.jpeg')
    assert not is_junction_file('junction06.jpg')
    assert not is_junction_file('IMG_junction01.jpg')
    assert not is_junction_file('20261106_071404305_iOS.jpg')


def test_build_prompt_detects_mission_clues_automatically():
    prompt = build_prompt()
    assert 'printed human figures' in prompt
    assert 'QR codes' in prompt
    assert '"mission_matches":[]' in prompt
    assert '"scene_type"' in prompt
    assert 'M01' not in prompt


def test_parse_plain_and_fenced():
    assert parse_scene_json('{"scene_type": "junction"}') == {
        'scene_type': 'junction'}
    fenced = 'Here:\n```json\n{"a": {"b": 1}}\n```'
    assert parse_scene_json(fenced) == {'a': {'b': 1}}


def test_parse_invalid():
    assert parse_scene_json(None) is None
    assert parse_scene_json('no json') is None
    assert parse_scene_json('{broken') is None
    assert parse_scene_json('[1, 2]') is None
