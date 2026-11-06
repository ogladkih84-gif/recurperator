#!/usr/bin/env python3
"""Build the diagnostic sketch with an installed AVR toolchain + Arduino AVR core."""
import os, pathlib, subprocess, tempfile, sys
root = pathlib.Path(__file__).resolve().parents[1]
prefix = pathlib.Path(os.environ.get('AVR_PREFIX', '/workspace/toolchains/avr/usr'))
corebase = pathlib.Path(os.environ.get('ARDUINO_AVR', str(prefix/'share/arduino/hardware/arduino/avr')))
core = corebase/'cores/arduino'
variant = corebase/'variants/eightanaloginputs'
build = pathlib.Path(tempfile.mkdtemp(prefix='bldc-avr-'))
flags = ['-mmcu=atmega328p', '-DF_CPU=16000000UL', '-DARDUINO=10819', '-DARDUINO_AVR_NANO', '-DARDUINO_ARCH_AVR', '-Os', '-ffunction-sections', '-fdata-sections', '-I'+str(core), '-I'+str(variant)]
objects = []
def compile_file(path, lang):
    obj = build/(path.name+'.o')
    compiler = prefix/'bin'/('avr-g++' if lang=='c++' else 'avr-gcc')
    extra = ['-std=gnu++11','-fno-exceptions','-fno-threadsafe-statics'] if lang=='c++' else []
    subprocess.run([str(compiler), *flags, *extra, '-x', lang, '-c', str(path), '-o', str(obj)], check=True)
    objects.append(str(obj))
for path in sorted(core.iterdir()):
    if path.suffix in ('.cpp','.c','.S'):
        compile_file(path, {'.cpp':'c++','.c':'c','.S':'assembler-with-cpp'}[path.suffix])
sketch = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv)>1 else root/'firmware/BMLC175A_v10_diag/BMLC175A_v10_diag.ino'
compile_file(sketch,'c++')
elf = build/(sketch.stem+'.elf')
subprocess.run([str(prefix/'bin/avr-g++'),'-mmcu=atmega328p','-Wl,--gc-sections',*objects,'-lm','-o',str(elf)],check=True)
subprocess.run([str(prefix/'bin/avr-size'),'--format=avr','--mcu=atmega328p',str(elf)],check=True)
subprocess.run([str(prefix/'bin/avr-objcopy'),'-O','ihex','-R','.eeprom',str(elf),str(elf.with_suffix('.hex'))],check=True)
print('Build artifacts:',build)
