import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
from threading import Thread, Lock
import struct
import time

class WirelessTuner:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.running = False
        self.lock = Lock()
        self.current_values = [0.0]*8
        self.init_ui()
        
    def init_ui(self):
        self.root.title("无线调参上位机")
        
        # 串口选择区
        self.port_frame = ttk.LabelFrame(self.root, text="串口设置")
        self.port_frame.pack(padx=10, pady=5, fill=tk.X)
        
        self.port_combobox = ttk.Combobox(self.port_frame)
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        
        self.baud_combobox = ttk.Combobox(self.port_frame, 
                                        values=["9600", "115200", "460800"])
        self.baud_combobox.set("460800")
        self.baud_combobox.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(self.port_frame, text="连接", 
                                    command=self.toggle_connect)
        self.connect_btn.pack(side=tk.RIGHT, padx=5)
        
        # 参数调节区
        self.param_frame = ttk.LabelFrame(self.root, text="调参通道")
        self.param_frame.pack(padx=10, pady=5, fill=tk.BOTH)
        
        self.sliders = []
        for i in range(8):
            frame = ttk.Frame(self.param_frame)
            frame.pack(fill=tk.X, pady=2)
            
            label = ttk.Label(frame, text=f"通道{i}:", width=8)
            label.pack(side=tk.LEFT)
            
            slider = ttk.Scale(frame, from_=0, to=3.3, 
                              command=lambda v,i=i: self.on_slider_change(i, v))
            slider.pack(side=tk.LEFT, expand=True, fill=tk.X)
            
            value_label = ttk.Label(frame, text="0.000", width=8)
            value_label.pack(side=tk.RIGHT)
            
            self.sliders.append((slider, value_label))
        
        # 波形显示区
        self.wave_frame = ttk.LabelFrame(self.root, text="波形显示")
        self.wave_frame.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        
        # 这里可以添加matplotlib绘图组件
        
        self.refresh_ports()
        
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.set(ports[0])
            
    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.stop()
        else:
            self.start()
            
    def start(self):
        port = self.port_combobox.get()
        baud = int(self.baud_combobox.get())
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.running = True
            Thread(target=self.read_loop, daemon=True).start()
            self.connect_btn.config(text="断开")
        except Exception as e:
            print("连接失败:", e)
            
    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()
        self.connect_btn.config(text="连接")
        
    def on_slider_change(self, channel, value):
        value = float(value)
        self.current_values[channel] = value
        slider, label = self.sliders[channel]
        label.config(text=f"{value:.3f}")
        self.send_parameter(channel, value)
        
    def send_parameter(self, channel, value):
        if not self.ser or not self.ser.is_open:
            return
            
        # 数据包格式：0xA5 + 通道号(1B) + 数值(float) + 0x5A
        packet = bytearray()
        packet.append(0xA5)
        packet.append(channel)
        packet.extend(struct.pack('<f', value))
        packet.append(0x5A)
        
        with self.lock:
            self.ser.write(packet)
            
    def read_loop(self):
        buffer = bytearray()
        while self.running:
            if self.ser.in_waiting:
                buffer.extend(self.ser.read(self.ser.in_waiting))
                
                # 查找完整数据包（示例协议：0xA5开头，0x5A结尾，20字节）
                while len(buffer) >= 20:
                    start = buffer.find(0xA5)
                    if start == -1:
                        buffer.clear()
                        break
                        
                    if len(buffer) - start < 20:
                        break
                        
                    packet = buffer[start:start+20]
                    if packet[-1] != 0x5A:
                        buffer = buffer[start+1:]
                        continue
                        
                    # 解析8个浮点数（示例协议）
                    try:
                        values = struct.unpack('<8f', packet[1:-1])
                        self.update_waveform(values)
                    except struct.error:
                        pass
                        
                    buffer = buffer[start+20:]
                    
            time.sleep(0.01)
            
    def update_waveform(self, values):
        # 这里添加波形更新逻辑
        print("收到波形数据:", values)
        
if __name__ == "__main__":
    root = tk.Tk()
    app = WirelessTuner(root)
    root.geometry("800x600")
    root.mainloop()