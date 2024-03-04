#!/usr/bin/env python

# this is a script used to generate ls2k_rom.h from ls2k.S

import re
import sys
import struct
import os
fh = open("ls2k_rom.h", "w")
fh.write("static unsigned int aui_boot_code[] = {\n")
os.system("loongarch64-linux-gnu-gcc -c ls2k.S")
data={}
code={}
f=os.popen("loongarch64-linux-gnu-objdump -j .text -dz ls2k.o")
l =f.readline()
while l:
  m = re.match(".*\s([0-9a-fA-F]+):\s+([0-9a-fA-F]{8}).*",l)
  if m:
      data[int(m.groups()[0],16)] = m.groups()[1]
  l =f.readline()
addr = 0
i = 0
f=os.popen("loongarch64-linux-gnu-gcc -c -Wa,-al,-am,-an ls2k.S")
l =f.readline()
while l:
  l = re.sub(' +', ' ', l.replace('\t', ' '))
  m0 = re.match(".*# \d+ .*", l)
  m = None if m0 else re.match(".*([0-9a-fA-F]{4}) ([0-9a-fA-F]{8})(.*)",l)
  m1 = None if m else re.match(".*\s+([0-9a-fA-F]{8})(.*)",l)
  m2 = None if m or m1 else re.match("\s+(\d+)\s+.*",l)
  if m0:
      pass
  elif m:
    i = int(m.groups()[0], 16)
    while addr < i:
        fh.write("0x"+data[addr]+",\n")
        addr += 4
    fh.write("0x"+data[addr]+", /* " + m.groups()[2] + " */\n")
    addr += 4
  elif m1:
    fh.write("0x"+data[addr]+",\n")
    addr += 4
  elif m2:
    fh.write("/*" + l.strip() + " */\n")
  l =f.readline()
fh.write("};\n")
fh.close()
