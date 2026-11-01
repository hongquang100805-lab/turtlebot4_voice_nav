"""Command semantics, complete-route rejection and unsafe ambiguity cases."""

import pytest
from turtlebot4_voice_nav.command_parser import normalize, parse_command

REGIONS = [
    {'id': 'A', 'name': 'Khu vực A', 'aliases': ['phòng học']},
    {'id': 'B', 'name': 'Khu vực B', 'aliases': ['phòng thí nghiệm']},
    {'id': 'C', 'name': 'Khu vực C', 'aliases': ['hành lang C', 'hành lang']},
]


@pytest.mark.parametrize(('text', 'route'), [
    ('Đi đến khu vực A.', ['A']),
    ('DI CHUYỂN ĐẾN KHU B', ['B']),
    ('Đi đến phòng thí nghiệm', ['B']),
    ('Di chuyển từ khu vực A đến khu vực B.', ['A', 'B']),
    ('Đi từ phòng A qua hành lang C rồi đến phòng B', ['A', 'C', 'B']),
    ('Đi đến khu vực A rồi đến khu vực C', ['A', 'C']),
    ('Robot hãy đi đến phòng học nhé', ['A']),
    ('Đi đến B', ['B']),
])
def test_routes(text, route):
    result = parse_command(text, REGIONS)
    assert result == {'intent': 'navigate', 'region_ids': route, 'text': text}
    assert parse_command(text, {'regions': REGIONS}) == result


@pytest.mark.parametrize('text', [
    'Đi đến khu vực D', 'Đi từ A đến D', 'Đi từ A', 'Đi đến',
    'Đi đến khu vực ABC', 'Đi đến nhà ăn', 'không đi đến khu A',
    'Đừng đi đến B', 'Tôi chưa muốn đi đến A', 'A', '', 'Đi từ A đến',
    'Đi đến A và có thể B', 'Hủy đi đến A',
])
def test_rejects_whole_command(text):
    with pytest.raises(ValueError):
        parse_command(text, REGIONS)


@pytest.mark.parametrize('text', ['Dừng lại', 'Hủy điều hướng', 'dừng robot', 'hủy lệnh'])
def test_cancel(text):
    assert parse_command(text, REGIONS)['intent'] == 'cancel'


def test_ambiguous_alias_is_not_guessed():
    duplicated = [*REGIONS, {'id': 'D', 'aliases': ['phòng học']}]
    with pytest.raises(ValueError, match='không rõ ràng'):
        parse_command('Đi đến phòng học', duplicated)


def test_identifier_does_not_match_inside_another_word():
    with pytest.raises(ValueError):
        parse_command('Đi đến ban', REGIONS)
    assert normalize('ĐI ĐẾN phòng thí nghiệm.') == 'di den phong thi nghiem'


@pytest.mark.parametrize(('text', 'route'), [
    ('Đi đến bê', ['B']),
    ('Đi đến khu vực bê', ['B']),
    ('Đi đến khu be', ['B']),
    ('Đi từ phòng A đến phòng bê', ['A', 'B']),
    ('Đi đến xê', ['C']),
    ('Đi đến cê', ['C']),
    ('Đi đến khu vực xê', ['C']),
    ('Đi đến phòng ce', ['C']),
    ('Đi từ A qua khu xê rồi đến bê', ['A', 'C', 'B']),
])
def test_spoken_region_letters(text, route):
    assert parse_command(text, REGIONS)['region_ids'] == route


@pytest.mark.parametrize('text', [
    'dung di den A', 'dung den khu be', 'dung toi C',
    'Đừng đi đến phòng bê', 'Đi đến bệnh viện', 'Đi đến xem',
])
def test_unaccented_negation_and_spoken_letter_boundaries(text):
    with pytest.raises(ValueError):
        parse_command(text, REGIONS)


def test_unaccented_stop_remains_cancel():
    assert parse_command('dung lai', REGIONS)['intent'] == 'cancel'


@pytest.mark.parametrize(('text', 'route'), [
    ('đi đến khua', ['A']),
    ('Đi từ khua đến khub', ['A', 'B']),
    ('Đi đến phongc', ['C']),
])
def test_exact_joined_place_prefix(text, route):
    assert parse_command(text, REGIONS)['region_ids'] == route


@pytest.mark.parametrize('text', ['Đi đến khuz', 'Đi từ khua đến khuz', 'Đi đến khuabc', 'Đi đến khuya'])
def test_joined_unknown_is_not_guessed(text):
    with pytest.raises(ValueError):
        parse_command(text, REGIONS)
