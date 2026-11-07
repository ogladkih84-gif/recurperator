"""Проверка выгрузки без подключения настоящего осциллографа."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'scripts/rigol_export.py'
spec = importlib.util.spec_from_file_location('rigol_export', MODULE)
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)

class Scope:
    def __init__(self, total=450123):
        self.total, self.channel, self.first, self.last = total, 3, 1, total
        self.blocks, self.commands = [], []
        self.truncate = False
    def write(self, command):
        self.commands.append(command)
        if command.startswith(':WAV:SOUR '): self.channel = int(command[-1])
        elif command.startswith(':WAV:STAR '): self.first = int(command.split()[-1])
        elif command.startswith(':WAV:STOP '): self.last = int(command.split()[-1])
    def query(self, command):
        if command == ':ACQ:MDEP?': return str(self.total)
        if command == ':WAV:PRE?':
            yinc, yorigin, yref = (.02, 7, 128) if self.channel == 3 else (.01, 3, 120)
            return f'0,2,{self.total},1,0.000001,-0.12,10,{yinc},{yorigin},{yref}'
        raise AssertionError(command)
    def query_binary_values(self, command, **kwargs):
        assert command == ':WAV:DATA?' and kwargs['datatype'] == 'B'
        assert 1 <= self.first <= self.last <= self.total
        assert self.last-self.first+1 <= 250000
        self.blocks.append((self.channel, self.first, self.last))
        data = bytes((i*(self.channel+2)+100)%256 for i in range(self.first-1,self.last))
        return data[:-1] if self.truncate else data

class ExportTests(unittest.TestCase):
    def test_chunk_alignment_scaling_and_csv(self):
        scope = Scope()
        a, ia = export.read_channel(scope, 3)
        b, ib = export.read_channel(scope, 4)
        self.assertEqual(ia['stride'], 3)
        self.assertEqual(len(a), 150041)
        self.assertEqual(a, bytes((i*5+100)%256 for i in range(0,scope.total,3)))
        self.assertEqual(b, bytes((i*6+100)%256 for i in range(0,scope.total,3)))
        self.assertEqual(len(scope.blocks), 6)
        self.assertEqual(sum(y-x+1 for ch,x,y in scope.blocks),scope.total*2)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'capture.csv'
            export.write_capture(path,a,ia,b,ib)
            with path.open() as f:
                rows=csv.DictReader(f)
                first=next(rows);second=next(rows)
                self.assertAlmostEqual(float(first['time_s']),-.12001)
                self.assertAlmostEqual(float(second['time_s'])-float(first['time_s']),3e-6)
                self.assertAlmostEqual(float(first['ch3_V']),(100-7-128)*.02)
                self.assertAlmostEqual(float(first['ch4_V']),(100-3-120)*.01)
                self.assertAlmostEqual(float(first['ch3_minus_ch4_V']),-.47)
    def test_truncated_transfer_rejected(self):
        scope=Scope(10000);scope.truncate=True
        with self.assertRaisesRegex(ValueError,'байт вместо'):
            export.read_channel(scope,3)
    def test_mismatched_time_rejected(self):
        scope=Scope(10000);a,ia=export.read_channel(scope,3);b,ib=export.read_channel(scope,4)
        ib['preamble']['xinc']*=2
        with self.assertRaisesRegex(ValueError,'Шаг времени'):
            export.write_capture('/unused.csv',a,ia,b,ib)
    def test_identity_and_preamble(self):
        self.assertTrue(export.is_supported('RIGOL TECHNOLOGIES,DS1104Z,example,00.04.04.SP4'))
        self.assertFalse(export.is_supported('OTHER,DS1104Z,example,1'))
        with self.assertRaises(ValueError): export.preamble_values('0,1,2')
        with self.assertRaises(ValueError): export.preamble_values('0,2,1200,1,0,0,0,1,0,0')
        self.assertEqual(export.volts(128,{'yorigin':0,'yref':128,'yinc':.01}),0)

if __name__=='__main__':
    unittest.main()
