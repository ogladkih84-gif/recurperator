#!/usr/bin/env python3
"""Выгрузка остановленной записи CH3/CH4 Rigol DS1000Z в общий CSV через VISA.

Требуется pyvisa и установленная системная VISA. Прошивка Nano не изменяется.
Прибор остаётся в STOP; RUN, очистка памяти и AUTO SCALE не используются.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re
import sys


def preamble_values(reply):
    """Разбираем десять полей WAV:PREamble семейства DS1000Z."""
    values = [float(v) for v in reply.strip().split(',')]
    if len(values) != 10:
        raise ValueError('Неожиданный ответ WAV:PREamble: ' + reply[:120])
    names = ('format', 'type', 'points', 'count', 'xinc', 'xorigin', 'xref', 'yinc', 'yorigin', 'yref')
    result = dict(zip(names, values))
    if not all(math.isfinite(v) for v in values) or result['xinc'] <= 0 or result['yinc'] <= 0:
        raise ValueError('Некорректные масштабы осциллограммы.')
    if result['format'] != 0:
        raise ValueError('Прибор не переключился в байтовый формат BYTE.')
    return result


def read_channel(scope, channel, max_points=200_000, chunk_points=200_000):
    """Читаем память блоками, сохраняя каждый N-й отсчёт на общей сетке."""
    if max_points <= 0 or not 1 <= chunk_points <= 250_000:
        raise ValueError('Неверный размер выгрузки.')
    scope.write(f':WAV:SOUR CHAN{channel}')
    scope.write(':WAV:MODE RAW')
    scope.write(':WAV:FORM BYTE')
    scope.write(':WAV:STAR 1')
    pre = preamble_values(scope.query(':WAV:PRE?'))
    depth_reply = scope.query(':ACQ:MDEP?').strip()
    try:
        depth_float = float(depth_reply)
        if not math.isfinite(depth_float) or depth_float != int(depth_float):
            raise ValueError('Некорректная глубина памяти.')
        total = int(depth_float)
    except ValueError:
        if depth_reply.upper() != 'AUTO':
            raise ValueError('Неожиданный ответ ACQ:MDEP: ' + depth_reply)
        total = int(pre['points'])
    if not 1 <= total <= 24_000_000:
        raise ValueError(f'Неподдерживаемая длина памяти DS1000Z: {total}')
    stride = max(1, math.ceil(total / max_points))
    saved = bytearray()
    for first in range(1, total + 1, chunk_points):
        last = min(total, first + chunk_points - 1)
        # Сначала уменьшаем STOP: это также позволяет безопасно вернуться к началу
        # после чтения другого канала с большим START в конце памяти.
        scope.write(':WAV:STAR 1')
        scope.write(f':WAV:STOP {last}')
        scope.write(f':WAV:STAR {first}')
        data = scope.query_binary_values(':WAV:DATA?', datatype='B', container=bytes,
                                         expect_termination=False)
        if len(data) != last - first + 1:
            raise ValueError(f'CH{channel}: получено {len(data)} байт вместо {last-first+1}.')
        offset = (-(first - 1)) % stride
        saved.extend(data[offset::stride])
    if len(saved) != (total + stride - 1) // stride:
        raise ValueError('Нарушена общая сетка отсчётов.')
    return bytes(saved), {'channel': channel, 'memory_points': total,
                         'stride': stride, 'preamble': pre, 'depth_reply': depth_reply}


def volts(raw, pre):
    """Формула DS1000Z: (BYTE − YORigin − YREFerence) × YINCrement."""
    return (raw - pre['yorigin'] - pre['yref']) * pre['yinc']


def write_capture(csv_path, ch3, info3, ch4, info4):
    """Проверяем совпадение времени двух каналов и сохраняем их разность."""
    p3, p4 = info3['preamble'], info4['preamble']
    if len(ch3) != len(ch4) or info3['stride'] != info4['stride'] or info3['memory_points'] != info4['memory_points']:
        raise ValueError('Длины записей каналов различаются; общая временная шкала не подтверждена.')
    if not math.isclose(p3['xinc'], p4['xinc'], rel_tol=1e-9, abs_tol=1e-15):
        raise ValueError('Шаг времени каналов различается.')
    start3 = p3['xorigin'] - p3['xref'] * p3['xinc']
    start4 = p4['xorigin'] - p4['xref'] * p4['xinc']
    if not math.isclose(start3, start4, rel_tol=1e-9, abs_tol=p3['xinc'] * 0.01):
        raise ValueError('Начало времени каналов различается.')
    with Path(csv_path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('time_s', 'ch3_V', 'ch4_V', 'ch3_minus_ch4_V'))
        for index, (a, b) in enumerate(zip(ch3, ch4)):
            t = start3 + index * info3['stride'] * p3['xinc']
            va, vb = volts(a, p3), volts(b, p4)
            writer.writerow((f'{t:.12g}', f'{va:.9g}', f'{vb:.9g}', f'{va-vb:.9g}'))


def is_supported(identity):
    fields = identity.strip().split(',')
    return len(fields) >= 2 and 'RIGOL' in fields[0].upper() and bool(re.fullmatch(r'DS1\d{3}Z(?:-.*)?', fields[1].upper()))


def open_scope(manager, resource=None):
    """Выбираем один поддерживаемый прибор; прочие приборы не перенастраиваем."""
    resources = [resource] if resource else list(manager.list_resources('USB?*INSTR'))
    matches, errors = [], []
    for address in resources:
        device = None
        try:
            device = manager.open_resource(address)
            device.timeout = 15_000
            device.write_termination = '\n'
            device.read_termination = None
            identity = device.query('*IDN?').strip()
            if is_supported(identity):
                matches.append((device, address, identity))
                device = None
            else:
                errors.append(f'{address}: другая модель — {identity}')
        except Exception as exc:
            errors.append(f'{address}: {exc}')
        finally:
            if device is not None:
                device.close()
    if len(matches) != 1:
        for device, _, _ in matches:
            device.close()
        detail = '\n'.join(errors)
        if len(matches) > 1:
            raise RuntimeError('Найдено несколько Rigol. Укажите нужный адрес через --resource.')
        raise RuntimeError('Прибор DS1000Z не найден. Закройте Ultra Sigma и проверьте VISA/USB.\n' + detail)
    return matches[0]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resource', help='Адрес VISA, если приборов несколько.')
    parser.add_argument('--visa-library', help='Путь к системной библиотеке VISA, если автоматический поиск не работает.')
    parser.add_argument('--max-points', type=int, default=200_000, help='Максимум строк CSV, по умолчанию 200000.')
    parser.add_argument('--output', type=Path, help='Новая папка для результата.')
    args = parser.parse_args(argv)
    if args.max_points < 1200:
        parser.error('--max-points должен быть не меньше 1200.')
    try:
        import pyvisa
    except ImportError:
        print('Установите зависимость: py -3 -m pip install "pyvisa>=1.14,<2"', file=sys.stderr)
        return 1
    manager = scope = None
    folder = args.output or Path.cwd() / ('rigol_capture_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    try:
        manager = pyvisa.ResourceManager(args.visa_library) if args.visa_library else pyvisa.ResourceManager()
        scope, address, identity = open_scope(manager, args.resource)
        print('Подключён:', identity)
        scope.chunk_size = 1024 * 1024
        scope.write(':STOP')
        if scope.query('*OPC?').strip() != '1':
            raise RuntimeError('Прибор не подтвердил выполнение команды остановки.')
        if scope.query(':TRIG:STAT?').strip().upper() != 'STOP':
            raise RuntimeError('Остановленное состояние записи не подтверждено.')
        print('Читаю память CH3…')
        data3, info3 = read_channel(scope, 3, args.max_points)
        print('Читаю память CH4…')
        data4, info4 = read_channel(scope, 4, args.max_points)
        folder.mkdir(parents=True, exist_ok=False)
        csv_path = folder / 'rigol_ch3_ch4.csv'
        write_capture(csv_path, data3, info3, data4, info4)
        metadata = {'identity': identity, 'resource': address,
                    'exported_at': datetime.now().astimezone().isoformat(),
                    'ch3': info3, 'ch4': info4, 'rows': len(data3),
                    'sample_interval_s': info3['preamble']['xinc'] * info3['stride'],
                    'sampling': 'Каждый N-й исходный отсчёт; фильтрация и усреднение не применялись.'}
        (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8')
        print('Сохранено:', csv_path.resolve())
        print('Строк:', len(data3), '; шаг, мкс:', metadata['sample_interval_s'] * 1e6)
        print('Пришлите rigol_ch3_ch4.csv и metadata.json из этой папки.')
        if metadata['sample_interval_s'] > 2e-6:
            print('Внимание: для исследования короткого плато ШИМ шаг грубоват; может понадобиться больше --max-points.')
        return 0
    except Exception as exc:
        print('Ошибка выгрузки:', exc, file=sys.stderr)
        print('Прибор не переводился в RUN. При ошибке VISA проверьте совместимость разрядности Python и установленной VISA.', file=sys.stderr)
        return 1
    finally:
        if scope is not None:
            scope.close()
        if manager is not None:
            manager.close()


if __name__ == '__main__':
    raise SystemExit(main())
