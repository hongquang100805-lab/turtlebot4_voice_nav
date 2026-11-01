"""Strict Vietnamese route grammar; unknown place names never become partial routes."""

import re
import unicodedata


def normalize(text):
    """Normalize accents/case while retaining word boundaries."""
    if not isinstance(text, str):
        raise ValueError('Câu lệnh phải là văn bản')
    text = unicodedata.normalize('NFD', text.lower().replace('đ', 'd'))
    text = ''.join(c for c in text if unicodedata.category(c) != 'Mn')
    return ' '.join(re.sub(r'[^a-z0-9\s]', ' ', text).split())


def _aliases(regions):
    if isinstance(regions, dict):
        regions = regions.get('regions')
    if not isinstance(regions, list) or not regions:
        raise ValueError('Danh sách khu vực rỗng hoặc không hợp lệ')
    aliases = {}
    for region in regions:
        identifier = str(region['id'])
        values = [identifier, region.get('name', identifier),
                  *region.get('aliases', []), f'khu vực {identifier}',
                  f'khu {identifier}', f'phòng {identifier}',
                  f'khu{identifier}', f'phong{identifier}']
        # Whisper often spells spoken letter names rather than Latin IDs.
        spoken_letters = {'B': ('bê',), 'C': ('xê', 'cê')}
        for spoken in spoken_letters.get(identifier.upper(), ()):
            values.extend([spoken, f'khu vực {spoken}', f'khu {spoken}',
                           f'phòng {spoken}'])
        for value in values:
            name = normalize(value)
            if name:
                aliases.setdefault(name, set()).add(identifier)
    return aliases


def parse_command(text, regions):
    """Return intent, complete ordered region_ids and original text, or ValueError.

    ``regions`` accepts either the YAML document or its list of region dictionaries.
    Supported grammar: đi đến X; di chuyển từ X đến Y; đi từ X qua Y rồi đến Z.
    Aliases must match an entire place expression, never a substring or fuzzy guess.
    """
    original = text
    command = normalize(text)
    if not command:
        raise ValueError('Câu lệnh rỗng')
    # Stop/cancel is explicit. Negated movement must not activate a positive route.
    if (re.search(r'\b(khong|chua)\b', command) or
            re.search(r'\bđừng\b', text.lower()) or
            re.search(r'\bdung\s+(?:di|den|toi)\b', command)):
        raise ValueError('Không thực hiện câu lệnh phủ định; hãy nói rõ đích đến')
    if command in {'dung', 'dung lai', 'dung robot', 'dung dieu huong',
                   'huy', 'huy lenh', 'huy dieu huong', 'huy hanh trinh'}:
        return {'intent': 'cancel', 'region_ids': [], 'text': original}
    prefix = re.match(r'^(?:robot\s+)?(?:hay\s+)?(?:di chuyen|di|den|toi)\b\s*', command)
    if not prefix:
        raise ValueError('Chưa có yêu cầu điều hướng: hãy nói Đi đến khu vực A')
    route = command[prefix.end():]
    from_route = bool(re.match(r'^tu\b', route))
    route = re.sub(r'^(?:tu|den|toi)\b\s*', '', route)
    route = re.sub(r'\s+(?:nhe|nha)$', '', route)
    aliases = _aliases(regions)
    # Prefer a full known name so connector words inside a name are not split.
    parts = [route] if route in aliases else re.split(
        r'\s+(?:(?:roi|sau do)\s+)?(?:qua|den|toi)\s+', route)
    if from_route and len(parts) < 2:
        raise ValueError('Lệnh từ khu vực phải có cả nơi xuất phát và nơi đến')
    if len(parts) > 8 or any(not part.strip() for part in parts):
        raise ValueError('Chuỗi điểm đến không hợp lệ (tối đa 8 điểm)')
    identifiers = []
    for part in parts:
        matches = aliases.get(part.strip(), set())
        if not matches:
            raise ValueError(f'Khu vực không tồn tại hoặc chưa được gắn nhãn: {part or "(trống)"}')
        if len(matches) != 1:
            raise ValueError(f'Tên khu vực không rõ ràng: {part}')
        identifiers.append(next(iter(matches)))
    return {'intent': 'navigate', 'region_ids': identifiers, 'text': original}
