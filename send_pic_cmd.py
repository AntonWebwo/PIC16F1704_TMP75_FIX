import smbus
import sys
import time
import json
import argparse

def c8(d): return sum(d) & 0xFF
def c16(d): return sum(d) & 0xFFFF

p = argparse.ArgumentParser(epilog="Author: Anton Vinogradov @vinantole support@minerflash.ru")
p.add_argument('-b','--bus',type=int,required=True)
p.add_argument('-a','--addr',type=str,required=True)
p.add_argument('-r','--read',type=int,default=0)
p.add_argument('-w','--write',nargs='+',default=[])
p.add_argument('--crc',type=int,choices=[8,16],default=8)
p.add_argument('--json',action='store_true')
args = p.parse_args()

bus = smbus.SMBus(args.bus)
addr = int(args.addr,16)
cmd = [int(x,16) for x in args.write]
size = len(cmd) + (2 if args.crc==8 else 3)
chk = c8([size]+cmd) if args.crc==8 else c16([size]+cmd)
pkt = [0x55,0xAA,size] + cmd + ([chk] if args.crc==8 else [chk>>8, chk&0xFF])

try:
    for b in pkt:
        bus.write_byte(addr, b)
        time.sleep(0.0005)
    r = []
    if args.read > 0:
        time.sleep(0.25)
        for _ in range(args.read):
            try:
                r.append(bus.read_byte(addr))
                time.sleep(0.0005)
            except: pass
        time.sleep(0.25)
    if r:
        res = " ".join(f"{b:02x}" for b in r)
        print(json.dumps({"status":"ok","response":res,"bytes":r}) if args.json else res)
    else:
        if args.write:
            print(json.dumps({"status":"ok","message":"Packet sent"}) if args.json else "")
except Exception as e:
    print(json.dumps({"status":"error","message":str(e)}) if args.json else f"Error: {e}", file=sys.stderr)
    sys.exit(1)
finally:
    bus.close()
