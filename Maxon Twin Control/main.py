"""
EPOS4 Dual Valve Tester – ctypes (SAFE, no slow zone)
Two independent channels (A & B), each with:
  - Connect/Disconnect/Stop, Interface/Port/Node/CPR
  - Manual Jog (CW/CCW hold), current ceiling
  - Set Closed / Set Open (manual limits)
  - Safety margin (%) clamping (slider + commands)
  - Override toggle for Full Close/Open
  - Slider (absolute % within [margin,100-margin])
  - Normal profile (RPM, Acc, Dec) with Apply + Read from EPOS (SDO 0x6081/83/84)
  - Optional Auto-move to a custom % after both limits set
No slow-zone logic; all motion uses the normal profile and is STRICTLY clamped to limits+margin.

Test each motor uncoupled first. Set CPR to your encoder counts/turn (e.g. 131072 for ENX 22 EMT 17-bit).
"""

import os, ctypes, ctypes.wintypes as wintypes, threading, time
from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk, messagebox

import os, sys

import os, sys

# Base directory (works for normal script AND PyInstaller .exe)
if getattr(sys, 'frozen', False):
    # running from bundled EXE
    BASE_DIR = sys._MEIPASS
else:
    # running from source
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Folder that contains all the EPOS DLLs
EPOS_DIR = os.path.join(BASE_DIR, "Config", "Epos config")
os.add_dll_directory(EPOS_DIR)

# Path to the main Maxon DLL
EPOS_DLL_PATH = os.path.join(EPOS_DIR, "EposCmd64.dll")


# ===== Defaults =====
DEFAULT_BAUDRATE  = 1_000_000
DEFAULT_CPR       = 131072  # ENX 22 EMT 17-bit (change if needed)
PROFILE_MAX_VEL_RPM = 800
PROFILE_ACC_RPM_S   = 4000
PROFILE_DEC_RPM_S   = 4000
LIMIT_MARGIN_PCT_DEF = 1.0
STATUS_POLL_HZ       = 10

CAL_RPM_DEF          = 500
CAL_ACC_DEF          = 200
CAL_DEC_DEF          = 20000
CAL_CURRENT_CEIL_DEF = 900

# ===== EPOS ctypes Wrapper =====
class EPOS:
    def __init__(self, dll_path: str):
        if not os.path.exists(dll_path):
            raise FileNotFoundError(dll_path)
        self.dll = ctypes.WinDLL(dll_path)
        self.key = None
        self.ec  = wintypes.DWORD()
        self._have = {}
        self._bind()

    def _bind(self):
        d=self.dll
        self.HANDLE=wintypes.HANDLE
        self.u16=ctypes.c_ushort; self.u32=ctypes.c_ulong; self.i32=ctypes.c_long; self.u8=ctypes.c_ubyte; self.cstr=ctypes.c_char_p
        def opt(n,a,r):
            try: f=getattr(d,n); f.argtypes=a; f.restype=r; self._have[n]=True; return f
            except AttributeError: self._have[n]=False; return None
        opt('VCS_OpenDevice',[self.cstr,self.cstr,self.cstr,self.cstr],self.HANDLE)
        opt('VCS_CloseDevice',[self.HANDLE],wintypes.BOOL)
        opt('VCS_GetErrorInfo',[self.u32,ctypes.c_char_p,self.u16],wintypes.BOOL)
        opt('VCS_SetProtocolStackSettings',[self.HANDLE,self.u32,self.u32,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_ClearFault',[self.HANDLE,self.u16,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_SetEnableState',[self.HANDLE,self.u16,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_SetDisableState',[self.HANDLE,self.u16,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_ActivateProfilePositionMode',[self.HANDLE,self.u16,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_MoveToPosition',[self.HANDLE,self.u16,self.i32,wintypes.BOOL,wintypes.BOOL,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_HaltPositionMovement',[self.HANDLE,self.u16,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_GetPositionIs',[self.HANDLE,self.u16,ctypes.POINTER(self.i32),ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_GetCurrentIs',[self.HANDLE,self.u16,ctypes.POINTER(ctypes.c_short),ctypes.POINTER(self.u32)],wintypes.BOOL)
        # profile funcs (may be missing)
        opt('VCS_SetPositionProfile',[self.HANDLE,self.u16,self.u32,self.u32,self.u32,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_SetMaxProfileVelocity',[self.HANDLE,self.u16,self.u32,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_SetMaxAcceleration',[self.HANDLE,self.u16,self.u32,ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_SetMaxDeceleration',[self.HANDLE,self.u16,self.u32,ctypes.POINTER(self.u32)],wintypes.BOOL)
        # SDO
        opt('VCS_SetObject',[self.HANDLE,self.u16,self.u16,self.u8,ctypes.c_void_p,self.u32,ctypes.POINTER(self.u32),ctypes.POINTER(self.u32)],wintypes.BOOL)
        opt('VCS_GetObject',[self.HANDLE,self.u16,self.u16,self.u8,ctypes.c_void_p,self.u32,ctypes.POINTER(self.u32),ctypes.POINTER(self.u32)],wintypes.BOOL)

    def _err(self):
        b=ctypes.create_string_buffer(200)
        self.dll.VCS_GetErrorInfo(self.ec.value,b,200)
        return b.value.decode(errors='ignore')

    def open(self, iface="USB", port="USB0", device="EPOS4", stack="MAXON SERIAL V2", baud=DEFAULT_BAUDRATE, timeout_ms=500):
        self.key=self.dll.VCS_OpenDevice(device.encode(),stack.encode(),iface.encode(),port.encode())
        if not self.key: raise RuntimeError("VCS_OpenDevice failed (wrong iface/port or busy)")
        if not self.dll.VCS_SetProtocolStackSettings(self.key,self.u32(baud),self.u32(timeout_ms),ctypes.byref(self.ec)):
            raise RuntimeError("SetProtocolStackSettings: "+self._err())

    def close(self):
        if self.key: self.dll.VCS_CloseDevice(self.key); self.key=None

    def clear_fault(self,node): self.dll.VCS_ClearFault(self.key,node,ctypes.byref(self.ec))
    def enable(self,node):
        if not self.dll.VCS_SetEnableState(self.key,node,ctypes.byref(self.ec)):
            raise RuntimeError("Enable: "+self._err())
    def disable(self,node): self.dll.VCS_SetDisableState(self.key,node,ctypes.byref(self.ec))
    def activate_ppm(self,node):
        if not self.dll.VCS_ActivateProfilePositionMode(self.key,node,ctypes.byref(self.ec)):
            raise RuntimeError("Activate PPM: "+self._err())
    def move_to(self,node,counts):
        if not self.dll.VCS_MoveToPosition(self.key,node,int(counts),wintypes.BOOL(True),wintypes.BOOL(True),ctypes.byref(self.ec)):
            raise RuntimeError("MoveToPosition: "+self._err())
    def halt_ppm(self,node): self.dll.VCS_HaltPositionMovement(self.key,node,ctypes.byref(self.ec))
    def pos_is(self,node):
        p=ctypes.c_long()
        if not self.dll.VCS_GetPositionIs(self.key,node,ctypes.byref(p),ctypes.byref(self.ec)):
            raise RuntimeError("GetPositionIs: "+self._err())
        return int(p.value)
    def cur_is(self,node):
        c=ctypes.c_short()
        if not self.dll.VCS_GetCurrentIs(self.key,node,ctypes.byref(c),ctypes.byref(self.ec)):
            raise RuntimeError("GetCurrentIs: "+self._err())
        return int(c.value)

    def set_ppm_profile(self,node,vel_rpm,acc_rpm_s,dec_rpm_s,cpr:int):
        ok=False
        if hasattr(self.dll,'VCS_SetPositionProfile'):
            if self.dll.VCS_SetPositionProfile(self.key,node,int(vel_rpm),int(acc_rpm_s),int(dec_rpm_s),ctypes.byref(self.ec)):
                ok=True
        if not ok:
            partial=False
            if hasattr(self.dll,'VCS_SetMaxProfileVelocity'):
                partial |= bool(self.dll.VCS_SetMaxProfileVelocity(self.key,node,int(vel_rpm),ctypes.byref(self.ec)))
            if hasattr(self.dll,'VCS_SetMaxAcceleration'):
                partial |= bool(self.dll.VCS_SetMaxAcceleration(self.key,node,int(acc_rpm_s),ctypes.byref(self.ec)))
            if hasattr(self.dll,'VCS_SetMaxDeceleration'):
                partial |= bool(self.dll.VCS_SetMaxDeceleration(self.key,node,int(dec_rpm_s),ctypes.byref(self.ec)))
            ok=partial
        if not ok and hasattr(self.dll,'VCS_SetObject'):
            def sdo_u32(idx,sub,val):
                v=self.u32(int(val)); sz=self.u32(ctypes.sizeof(v))
                if not self.dll.VCS_SetObject(self.key,node,idx,sub,ctypes.byref(v),sz,ctypes.byref(self.ec),ctypes.byref(self.ec)):
                    raise RuntimeError(f"SDO 0x{idx:04X}:{sub:02X}: "+self._err())
            cpr=max(1,int(cpr))
            sdo_u32(0x6081,0x00, int(max(1,round(vel_rpm*cpr/60.0))))
            sdo_u32(0x6083,0x00, int(max(1,round(acc_rpm_s*cpr/60.0))))
            sdo_u32(0x6084,0x00, int(max(1,round(dec_rpm_s*cpr/60.0))))
            ok=True
        if not ok: raise RuntimeError("Unable to set profile")

    def get_ppm_profile_sdo(self,node,cpr:int):
        if not hasattr(self.dll,'VCS_GetObject'):
            raise RuntimeError("VCS_GetObject not available")
        def get_u32(idx,sub):
            buf=(ctypes.c_ubyte*4)(); out=wintypes.DWORD(0)
            if not self.dll.VCS_GetObject(self.key,node,idx,sub,ctypes.byref(buf),wintypes.DWORD(4),ctypes.byref(out),ctypes.byref(self.ec)):
                raise RuntimeError(f"SDO read 0x{idx:04X}:{sub:02X}: "+self._err())
            return int(ctypes.c_uint32.from_buffer(buf).value)
        cpr=max(1,int(cpr))
        vel=get_u32(0x6081,0x00); acc=get_u32(0x6083,0x00); dec=get_u32(0x6084,0x00)
        return vel*60.0/cpr, acc*60.0/cpr, dec*60.0/cpr

# ===== Limits mapping =====
@dataclass
class ValveMap:
    closed_counts: int | None = None
    open_counts:   int | None = None
    def have_limits(self)->bool:
        return self.closed_counts is not None and self.open_counts is not None and self.closed_counts!=self.open_counts
    def pct_to_counts(self,pct:float)->int:
        if not self.have_limits(): return 0
        c,o=self.closed_counts,self.open_counts
        pct=max(0.0,min(100.0,float(pct)))
        return int(round(c + (pct/100.0)*(o-c)))
    def counts_to_pct(self,counts:int)->float:
        if not self.have_limits(): return 0.0
        c,o=self.closed_counts,self.open_counts
        val=0.0 if o==c else 100.0*(counts-c)/(o-c)
        return max(0.0,min(100.0,val))
    def clamp_counts(self,counts:int,margin_frac:float)->int:
        if not self.have_limits(): return counts
        c,o=self.closed_counts,self.open_counts
        lo,hi=(c,o) if c<=o else (o,c)
        span=hi-lo; m=int(span*max(0.0,float(margin_frac)))
        lo+=m; hi-=m
        return max(lo,min(hi,counts))
    def endpoints_pct(self, m:float):
        m=max(0.0,min(100.0,float(m))); return (m, 100.0-m)

# ===== Single Motor Panel =====
class MotorPanel(ttk.LabelFrame):
    def __init__(self, master, title:str, default_iface:str, default_port:str, default_node:int):
        super().__init__(master, text=title, padding=8)
        self.epos=None
        self.iface=tk.StringVar(value=default_iface)
        self.port=tk.StringVar(value=default_port)
        self.node=tk.IntVar(value=default_node)
        self.cpr=tk.IntVar(value=DEFAULT_CPR)

        self.map=ValveMap()
        self.margin=tk.DoubleVar(value=LIMIT_MARGIN_PCT_DEF)
        self.override=tk.BooleanVar(value=False)

        self.rpm=tk.DoubleVar(value=PROFILE_MAX_VEL_RPM)
        self.acc=tk.DoubleVar(value=PROFILE_ACC_RPM_S)
        self.dec=tk.DoubleVar(value=PROFILE_DEC_RPM_S)

        self.cal_rpm=tk.DoubleVar(value=CAL_RPM_DEF)
        self.cal_acc=tk.DoubleVar(value=CAL_ACC_DEF)
        self.cal_dec=tk.DoubleVar(value=CAL_DEC_DEF)
        self.cal_ceil=tk.IntVar(value=CAL_CURRENT_CEIL_DEF)

        self.auto_center=tk.BooleanVar(value=True)
        self.auto_center_pct=tk.DoubleVar(value=50.0)

        self.follow=tk.BooleanVar(value=True)
        self.target_pct=tk.DoubleVar(value=0.0)
        self.cur_pct=tk.DoubleVar(value=0.0)
        self.cur_mA=tk.IntVar(value=0)

        self.connected=False; self.polling=False
        self._drag=False; self._sync=False; self._jog=False

        self._build_ui()
        self._lock_controls_until_limits()
        self.margin.trace_add("write", lambda *_: self._update_slider_bounds())

    # ---- UI ----
    def _build_ui(self):
        style = ttk.Style()
        style.configure("Big.TButton", font=("Arial", 20))  
        self.columnconfigure(0,weight=1)
        top=ttk.Frame(self); top.grid(row=0,column=0,sticky="ew")
        for i in range(12): top.columnconfigure(i,weight=1)
        ttk.Label(top,text="Iface").grid(row=0,column=0,sticky="w")
        ttk.Combobox(top,textvariable=self.iface,values=["USB","RS232","CANopen"],width=8,state="readonly").grid(row=0,column=1,sticky="w")
        ttk.Label(top,text="Port").grid(row=0,column=2,sticky="w"); ttk.Entry(top,textvariable=self.port,width=10).grid(row=0,column=3,sticky="w")
        ttk.Label(top,text="Node").grid(row=0,column=4,sticky="w"); ttk.Spinbox(top,from_=1,to=127,textvariable=self.node,width=6).grid(row=0,column=5,sticky="w")
        ttk.Label(top,text="CPR").grid(row=0,column=6,sticky="w"); ttk.Spinbox(top,from_=100,to=200000,increment=10,textvariable=self.cpr,width=9).grid(row=0,column=7,sticky="w")
        ttk.Button(top,text="Connect",command=self.connect).grid(row=0,column=8,sticky="e")
        ttk.Button(top,text="Disconnect",command=self.disconnect).grid(row=0,column=9,sticky="e")
        ttk.Button(top,text="Stop",command=self.stop_all).grid(row=0,column=10,sticky="e")

        self.ctrl=ttk.LabelFrame(self,text="Control (slider)")
        self.ctrl.grid(row=1,column=0,sticky="ew",pady=(8,4))
        self.ctrl.columnconfigure(0,weight=1); self.ctrl.columnconfigure(1,weight=0)
        ttk.Label(self.ctrl,text="Target opening (%)").grid(row=0,column=0,sticky="w")
        self.scale=ttk.Scale(self.ctrl,from_=0,to=100,variable=self.target_pct)
        self.scale.grid(row=1,column=0,sticky="ew",padx=5)
        self.ctrl.bind("<Configure>", lambda e: self.scale.configure(length=max(200,self.ctrl.winfo_width()-80)))
        self.scale.bind("<ButtonPress-1>", lambda e: self._begin_drag())
        self.scale.bind("<B1-Motion>", lambda e: self._update_target_label())
        self.scale.bind("<ButtonRelease-1>", lambda e: self._commit_slider())
        self.lbl_target=ttk.Label(self.ctrl,text="0.0 %"); self.lbl_target.grid(row=1,column=1,padx=8,sticky="e")
        ttk.Checkbutton(self.ctrl,text="Slider follows actual",variable=self.follow).grid(row=2,column=0,sticky="w",pady=(4,0))

        safety=ttk.Frame(self.ctrl); safety.grid(row=3,column=0,columnspan=2,sticky="ew",pady=(6,0))
        ttk.Label(safety,text="Safety margin (%)").grid(row=0,column=0,sticky="w")
        ttk.Spinbox(safety,from_=0.0,to=10.0,increment=0.5,textvariable=self.margin,width=8).grid(row=0,column=1,sticky="w",padx=(6,12))
        ttk.Checkbutton(safety,text="Override for Full Open/Close",variable=self.override).grid(row=0,column=2,sticky="w",padx=(6,0))

        fc=ttk.Frame(self.ctrl); fc.grid(row=4,column=0,columnspan=2,sticky="w",pady=(6,0))
        ttk.Button(fc,text="Fully Close (0%)",command=self.full_close).grid(row=0,column=0,padx=(0,8))
        ttk.Button(fc,text="Fully Open (100%)",command=self.full_open).grid(row=0,column=1)

        right=ttk.Frame(self); right.grid(row=2,column=0,sticky="ew",pady=(6,0))
        prof=ttk.LabelFrame(right,text="Normal profile (RPM)"); prof.grid(row=0,column=0,sticky="ew")
        for i in range(3): prof.columnconfigure(i,weight=1)
        ttk.Label(prof,text="RPM max").grid(row=0,column=0,sticky="w")
        ttk.Spinbox(prof,from_=10,to=8000,increment=10,textvariable=self.rpm,width=10).grid(row=0,column=1,sticky="e")
        ttk.Button(prof,text="Apply",command=self.apply_profile).grid(row=0,column=2,sticky="e")
        ttk.Label(prof,text="Acc (rpm/s)").grid(row=1,column=0,sticky="w")
        ttk.Spinbox(prof,from_=10,to=50000,increment=50,textvariable=self.acc,width=10).grid(row=1,column=1,sticky="e")
        ttk.Button(prof,text="Read EPOS",command=self.read_profile).grid(row=1,column=2,sticky="e")
        ttk.Label(prof,text="Dec (rpm/s)").grid(row=2,column=0,sticky="w")
        ttk.Spinbox(prof,from_=10,to=50000,increment=50,textvariable=self.dec,width=10).grid(row=2,column=1,sticky="e")

        cal=ttk.LabelFrame(self,text="Manual calibration – Jog & Limits")
        cal.grid(row=3,column=0,sticky="ew",pady=(6,0))
        for i in range(3): cal.columnconfigure(i,weight=1)
        ttk.Label(cal,text="Cal RPM").grid(row=0,column=0,sticky="w")
        ttk.Spinbox(cal,from_=3,to=1000,increment=1,textvariable=self.cal_rpm,width=10).grid(row=0,column=1,sticky="e")
        ttk.Label(cal,text="Cal Acc (rpm/s)").grid(row=1,column=0,sticky="w")
        ttk.Spinbox(cal,from_=10,to=50000,increment=50,textvariable=self.cal_acc,width=10).grid(row=1,column=1,sticky="e")
        ttk.Label(cal,text="Cal Dec (rpm/s)").grid(row=2,column=0,sticky="w")
        ttk.Spinbox(cal,from_=10,to=50000,increment=50,textvariable=self.cal_dec,width=10).grid(row=2,column=1,sticky="e")
        ttk.Label(cal,text="Current ceiling (mA)").grid(row=3,column=0,sticky="w")
        ttk.Spinbox(cal,from_=200,to=4000,increment=50,textvariable=self.cal_ceil,width=10).grid(row=3,column=1,sticky="e")
        jog=ttk.Frame(cal); jog.grid(row=4,column=0,columnspan=3,pady=(6,6),sticky="w")
        b1=ttk.Button(jog,text="⬅",width=8,style="Big.TButton"); b2=ttk.Button(jog,text="⮕",width=8,style="Big.TButton") # Open Close buttons
        b1.grid(row=0,column=0,padx=(0,8)); b2.grid(row=0,column=1)
        b1.bind("<ButtonPress-1>", lambda e: self.start_jog(-1))
        b1.bind("<ButtonRelease-1>", lambda e: self.stop_jog())
        b2.bind("<ButtonPress-1>", lambda e: self.start_jog(+1))
        b2.bind("<ButtonRelease-1>", lambda e: self.stop_jog())
        ttk.Button(cal,text="Set current position as Closed",command=self.set_closed).grid(row=5,column=0,columnspan=2,sticky="ew",pady=(2,2))
        ttk.Button(cal,text="Set current position as Open",command=self.set_open).grid(row=6,column=0,columnspan=2,sticky="ew",pady=(2,6))

        auto=ttk.LabelFrame(self,text="Auto-move after limits")
        auto.grid(row=4,column=0,sticky="ew")
        ttk.Checkbutton(auto,text="Enable",variable=self.auto_center).grid(row=0,column=0,sticky="w")
        ttk.Label(auto,text="Target (%)").grid(row=0,column=1,sticky="e",padx=(12,6))
        ttk.Spinbox(auto,from_=0.0,to=100.0,increment=0.5,textvariable=self.auto_center_pct,width=8).grid(row=0,column=2,sticky="w")

        stat=ttk.LabelFrame(self,text="Status")
        stat.grid(row=5,column=0,sticky="ew",pady=(6,0))
        for i in range(3): stat.columnconfigure(i,weight=1)
        ttk.Label(stat,text="Actual position (%)").grid(row=0,column=0,sticky="w")
        ttk.Label(stat,textvariable=self.cur_pct).grid(row=0,column=1,sticky="w")
        ttk.Label(stat,text="Current (mA)").grid(row=1,column=0,sticky="w")
        ttk.Label(stat,textvariable=self.cur_mA).grid(row=1,column=1,sticky="w")
        self.lbl_limits=ttk.Label(stat,text="Closed: —  |  Open: —  (counts)")
        self.lbl_limits.grid(row=2,column=0,columnspan=3,sticky="w")

    # ---- Enable/disable slider when no limits
    def _lock_controls_until_limits(self):
        state=("normal" if self.map.have_limits() else "disabled")
        for w in self.ctrl.winfo_children():
            try: w.configure(state=state)
            except Exception: pass
        # keep the follow checkbox active
        for w in self.ctrl.winfo_children():
            if isinstance(w, ttk.Checkbutton):
                try: w.configure(state="normal")
                except Exception: pass
        self._update_slider_bounds()

    def _update_slider_bounds(self):
        lo,hi=(0.0,100.0)
        if self.map.have_limits(): lo,hi=self.map.endpoints_pct(self.margin.get())
        self.scale.configure(from_=lo,to=hi)
        v=max(lo,min(hi,float(self.target_pct.get())))
        self.target_pct.set(v); self._update_target_label()

    # ---- EPOS ops
    def connect(self):
        try:
            self.epos=EPOS(EPOS_DLL_PATH)
            self.epos.open(iface=self.iface.get(),port=self.port.get())
            self.epos.clear_fault(self.node.get()); self.epos.enable(self.node.get()); self.epos.activate_ppm(self.node.get())
            self.apply_profile()
            self.connected=True; self._start_poll()
            messagebox.showinfo(self['text'],"Connected.")
        except Exception as e: messagebox.showerror(self['text'],str(e))

    def disconnect(self):
        self.connected=False; self._stop_poll()
        try:
            if self.epos:
                self.stop_jog()
                self.epos.disable(self.node.get()); self.epos.close(); self.epos=None
        except Exception as e: messagebox.showerror(self['text'],str(e))

    def apply_profile(self):
        if not self.epos: return
        try: self.epos.set_ppm_profile(self.node.get(), self.rpm.get(), self.acc.get(), self.dec.get(), self.cpr.get())
        except Exception as e: messagebox.showerror(self['text']+" – Apply profile",str(e))

    def read_profile(self):
        if not self.epos: return
        try:
            v,a,d=self.epos.get_ppm_profile_sdo(self.node.get(), self.cpr.get())
            self.rpm.set(round(v,1)); self.acc.set(round(a,1)); self.dec.set(round(d,1))
        except Exception as e: messagebox.showerror(self['text']+" – Read EPOS",str(e))

    def stop_all(self):
        if self.epos:
            try: self.epos.halt_ppm(self.node.get())
            except Exception as e: messagebox.showerror(self['text']+" – Stop",str(e))

    # ---- Polling
    def _start_poll(self):
        if self.polling: return
        self.polling=True; threading.Thread(target=self._poll,daemon=True).start()
    def _stop_poll(self): self.polling=False
    def _poll(self):
        while self.polling and self.epos:
            try:
                p=self.epos.pos_is(self.node.get()); c=self.epos.cur_is(self.node.get())
                act=self.map.counts_to_pct(p)
                self.cur_pct.set(round(act,2)); self.cur_mA.set(c)
                if self.follow.get() and not self._drag:
                    lo=float(self.scale.cget("from")); hi=float(self.scale.cget("to"))
                    vis=max(lo,min(hi,act))
                    self._sync=True
                    try: self.target_pct.set(vis)
                    finally: self._sync=False
                    self._update_target_label()
            except Exception:
                pass
            time.sleep(1/STATUS_POLL_HZ)

    # ---- Jog / limits
    def start_jog(self, direction:int):
        if not (self.connected and self.epos): return
        node=self.node.get()
        rpm=max(3,int(self.cal_rpm.get()))*(1 if direction>0 else -1)
        acc=max(10,int(self.cal_acc.get())); dec=max(10,int(self.cal_dec.get()))
        ceil=int(self.cal_ceil.get())
        self._jog=True
        def worker():
            try:
                self.epos.activate_ppm(node)  # ensure mode chain ok
                # use velocity mode for jog
                if hasattr(self.epos.dll,'VCS_ActivateProfileVelocityMode'):
                    self.epos.dll.VCS_ActivateProfileVelocityMode(self.epos.key,node,ctypes.byref(self.epos.ec))
                if hasattr(self.epos.dll,'VCS_SetVelocityProfile'):
                    self.epos.dll.VCS_SetVelocityProfile(self.epos.key,node,int(acc),int(dec),ctypes.byref(self.epos.ec))
                if hasattr(self.epos.dll,'VCS_MoveWithVelocity'):
                    self.epos.dll.VCS_MoveWithVelocity(self.epos.key,node,int(rpm),ctypes.byref(self.epos.ec))
                while self._jog and self.epos:
                    if abs(self.epos.cur_is(node))>=ceil:
                        if hasattr(self.epos.dll,'VCS_HaltVelocityMovement'):
                            self.epos.dll.VCS_HaltVelocityMovement(self.epos.key,node,ctypes.byref(self.epos.ec))
                        self._jog=False
                        messagebox.showwarning(self['text'],"Jog stopped: current ceiling reached.")
                        break
                    time.sleep(0.01)
            except Exception as e:
                messagebox.showerror(self['text']+" – Jog",str(e))
            finally:
                try:
                    self.epos.activate_ppm(node); self.apply_profile()
                except Exception: pass
        threading.Thread(target=worker,daemon=True).start()

    def stop_jog(self):
        if self._jog and self.epos:
            self._jog=False
            try:
                if hasattr(self.epos.dll,'VCS_HaltVelocityMovement'):
                    self.epos.dll.VCS_HaltVelocityMovement(self.epos.key,self.node.get(),ctypes.byref(self.epos.ec))
            except Exception: pass
            try:
                self.epos.activate_ppm(self.node.get()); self.apply_profile()
            except Exception: pass

    def set_closed(self):
        if not self.epos: return
        self.map.closed_counts=self.epos.pos_is(self.node.get())
        self._after_limit_change(0.0)

    def set_open(self):
        if not self.epos: return
        self.map.open_counts=self.epos.pos_is(self.node.get())
        self._after_limit_change(100.0)

    def _after_limit_change(self, slider_to:float):
        self.lbl_limits.config(text=f"Closed: {self.map.closed_counts if self.map.closed_counts is not None else '—'}  |  Open: {self.map.open_counts if self.map.open_counts is not None else '—'} (counts)")
        self._lock_controls_until_limits()
        if self.map.have_limits():
            if self.auto_center.get():
                lo,hi=self.map.endpoints_pct(self.margin.get())
                pct=max(lo,min(hi,float(self.auto_center_pct.get())))
                try:
                    self._move_to_pct_clamped(pct)
                    self._sync=True; self.target_pct.set(pct); self._sync=False; self._update_target_label()
                except Exception as e:
                    messagebox.showwarning(self['text']+" – Auto-move",str(e))
            else:
                lo,hi=self.map.endpoints_pct(self.margin.get())
                vis=max(lo,min(hi,slider_to))
                self._sync=True; self.target_pct.set(vis); self._sync=False; self._update_target_label()

    # ---- Slider commands
    def _begin_drag(self): self._drag=True
    def _update_target_label(self): self.lbl_target.config(text=f"{self.target_pct.get():.1f} %")
    def _commit_slider(self):
        self._drag=False
        if self._sync or not (self.connected and self.epos and self.map.have_limits()): return
        try: self._move_to_pct_clamped(float(self.target_pct.get()))
        except Exception as e: messagebox.showerror(self['text']+" – Slider cmd",str(e))

    # ---- Full Open/Close
    def full_close(self):
        if not (self.connected and self.epos and self.map.have_limits()): return
        try:
            if self.override.get():
                tgt=self.map.closed_counts
            else:
                pct=self.map.endpoints_pct(self.margin.get())[0]
                tgt=self._counts_for_pct(pct, use_margin=True)
            self.epos.move_to(self.node.get(), tgt)
        except Exception as e: messagebox.showerror(self['text']+" – Full Close",str(e))

    def full_open(self):
        if not (self.connected and self.epos and self.map.have_limits()): return
        try:
            if self.override.get():
                tgt=self.map.open_counts
            else:
                pct=self.map.endpoints_pct(self.margin.get())[1]
                tgt=self._counts_for_pct(pct, use_margin=True)
            self.epos.move_to(self.node.get(), tgt)
        except Exception as e: messagebox.showerror(self['text']+" – Full Open",str(e))

    # ---- Helpers
    def _move_to_pct_clamped(self,pct:float):
        lo=float(self.scale.cget("from")); hi=float(self.scale.cget("to"))
        pct=max(lo,min(hi,float(pct)))
        counts=self._counts_for_pct(pct, use_margin=True)
        self.epos.move_to(self.node.get(), counts)

    def _counts_for_pct(self,pct:float,use_margin:bool)->int:
        counts=self.map.pct_to_counts(max(0.0,min(100.0,pct)))
        if use_margin:
            margin_frac=max(0.0,float(self.margin.get())/100.0)
            counts=self.map.clamp_counts(counts,margin_frac)
        # Absolute clamp again for safety
        c,o=self.map.closed_counts,self.map.open_counts
        lo=min(c,o); hi=max(c,o)
        return max(lo,min(hi,counts))

# ===== Main App with two panels =====
class DualApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dual Maxon Tester – ctypes (EPOS4)")
        self.geometry("1250x900"); self.minsize(1200,820)

        # 🔹 just add this:
        style = ttk.Style(self)
        style.configure("TLabelframe.Label", font=("Segoe UI", 12, "bold"))

        container=ttk.Frame(self,padding=10); container.pack(fill="both",expand=True)
        container.columnconfigure(0,weight=1); container.columnconfigure(1,weight=1)

        self.panelA=MotorPanel(container,"Motor A", default_iface="USB", default_port="USB0", default_node=2)
        self.panelB=MotorPanel(container,"Motor B", default_iface="USB", default_port="USB1", default_node=2)

        self.panelA.grid(row=0,column=0,sticky="nsew",padx=(0,8))
        self.panelB.grid(row=0,column=1,sticky="nsew",padx=(8,0))


if __name__=="__main__":
    app=DualApp(); app.mainloop()
