import ctypes as c,re,subprocess
from PIL import Image
x=c.CDLL('libX11.so.6');x.XOpenDisplay.restype=c.c_void_p;d=x.XOpenDisplay(None)
class XI(c.Structure):
 _fields_=[('width',c.c_int),('height',c.c_int),('xoffset',c.c_int),('format',c.c_int),('data',c.c_void_p),('byte_order',c.c_int),('bitmap_unit',c.c_int),('bitmap_bit_order',c.c_int),('bitmap_pad',c.c_int),('depth',c.c_int),('bytes_per_line',c.c_int),('bits_per_pixel',c.c_int)]
s=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True).stdout
m=re.search(r'(0x[0-9a-f]+) "[^"\n]* - RViz"[^\n]*? (\d+)x(\d+)\+',s)
if not m:raise SystemExit('RViz window unavailable')
w,width,height=int(m[1],16),int(m[2]),int(m[3]);x.XGetImage.argtypes=[c.c_void_p,c.c_ulong,c.c_int,c.c_int,c.c_uint,c.c_uint,c.c_ulong,c.c_int];x.XGetImage.restype=c.POINTER(XI)
p=x.XGetImage(d,w,0,0,width,height,c.c_ulong(-1).value,2)
if not p:raise SystemExit('Could not capture window')
v=p.contents;b=c.string_at(v.data,v.bytes_per_line*v.height)
Image.frombytes('RGB',(v.width,v.height),b,'raw','BGRX',v.bytes_per_line).save(str(__import__('pathlib').Path(__file__).resolve().parents[1]/'logs/rviz-live.png'))
print('RViz window captured',v.width,v.height)
