# exp1: Motor car control
import serial
import serial.tools.list_ports
import struct
import time
import threading
import queue
from dataclasses import dataclass

# ├────────────────────────────────────────────
# │ 数据类
# ├────────────────────────────────────────────
@dataclass
class MotorState:
    """电机系统状态数据类"""
    voltage: float = 0.0      # 总线电压 (V)
    current: float = 0.0      # 实时相电流 (mA)
    speed: float = 0.0        # 实时转速 (rpm)
    angle: float = 0.0        # 实时位置角度
    encoder: int = 0          # 编码器值 (0-65535)
    motor_state: int = 0      # 电机状态标志位
    homing_state: int = 0     # 回零状态标志位

# ├────────────────────────────────────────────
# │ class Motor_ctl
# ├────────────────────────────────────────────
class Motor_ctl:
    # ├──── 常量区 ────
    device_name = 'USB-SERIAL CH340'
    BAUD = 115200 
    
    spd_ctl = [0x01, 0x10, 0x00, 0xF6, 0x00, 0x03, 0x06, 0x00, 0x0a, 0x00, 0x80, 0x00, 0x00]
    sync_motion = [0x00, 0x10, 0x00, 0xFF, 0x00, 0x01, 0x02, 0x66, 0x00] 
    stop = [0x00, 0x10, 0x00, 0xFE, 0x00, 0x01, 0x02, 0x98, 0x00] 
    readparam = [0x01, 0x04, 0x00, 0x42, 0x00, 0x0f, 0x11, 0x9e]
    writeparam = [0x01, 0x10, 0x00, 0x48, 0x00, 0x0f, 0x1e] 
    setang = [0x01, 0x10, 0x00, 0xFD, 0x00, 0x05, 0x0A, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0c, 0x80, 0x00, 0x00]
    
    mode_switch = [0x01, 0x10, 0x00, 0x46, 0x00, 0x02, 0x04, 0x69, 0x00, 0x02, 0x00] 
    enable_ctl = [0x01, 0x10, 0x00, 0xF3, 0x00, 0x02, 0x04, 0xAB, 0x01, 0x00, 0x00] 
    clear_pos = [0x01, 0x06, 0x00, 0x0A, 0x00, 0x01] 

    MAXRPM = 255                
    MAXCMDSIZE = 20             
    CRC16_POLYNOMIAL = 0xa001   
    MOTOR_DIR = {
        0x01: 0,
        0x03: 0,
        0x02: 1,  
        0x04: 1
    }
    TOP_MOTOR_ID = 3  # 预留的防缠绕顶层电机ID
    
    ctl_stop_event = threading.Event()  
    query_event = threading.Event()      

    # ├──── 初始化 ────
    def __init__(self, speed=36):
        self.thread = None
        self.ser = None
        self.minspd = 5
        self.maxspd = 160
        self.spd = speed
        self.acc = 0x10
        self.turn = 0.5
        self.task_queue = queue.Queue(maxsize=self.MAXCMDSIZE)

    def find_port_by_name(self, device_name):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if device_name.lower() in port.description.lower():
                return port.device
        return None

    def init_motor(self, port, baudrate):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1) # 超时稍微放大便于读取长数据
            self.thread = threading.Thread(target=self.process_task)
            self.thread.daemon = True 
            self.thread.start()  
        except serial.SerialException as e:
            print(f"串口初始化失败: {e}")

    # ├──── 串口通信层 ────
    def calculate_crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = ((crc >> 1) ^ self.CRC16_POLYNOMIAL) & 0xFFFF
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def create_cmd(self, data):
        if isinstance(data, list):
            data_short = bytes(data)  
        else:
            data_short = bytes(data)
        crc = self.calculate_crc16(data_short)
        crc_bytes = struct.pack('<H', crc)
        return data_short + crc_bytes

    def send_cmd(self, bytes_data, read_response=False):
        """
        统一的发送指令口
        :param read_response: 是否需要阻塞读取返回的 payload
        """
        if self.ser and self.ser.is_open:
            try:
                # 1. 读指令发送前，必须清空接收缓冲，防止上次残留的数据干扰
                if read_response:
                    self.ser.reset_input_buffer()
                
                self.ser.write(bytes_data)
                
                if read_response:
                    # 2. 给予硬件足够的响应和传输时间 (115200波特率下37字节约需3.5ms)
                    time.sleep(0.015) 
                    
                    resp = b""
                    start_time = time.time()
                    # 3. 设定一个 50ms 的硬超时，防止无限死锁
                    while (time.time() - start_time) < 0.05:
                        if self.ser.in_waiting:
                            resp += self.ser.read(self.ser.in_waiting)
                        time.sleep(0.002)
                    return resp
                else:
                    time.sleep(0.01) # 普通写指令的短暂延时
                    return True
            except serial.SerialException as e:
                print(f"[错误] 发送失败: {e}")
                return False
        else:
            print("[警告] 串口未开启")
            return False

    def read_registers(self, id, reg_addr, reg_num):
        """04H 功能码通用读取接口"""
        cmdbin = [id, 0x04, (reg_addr >> 8) & 0xFF, reg_addr & 0xFF, (reg_num >> 8) & 0xFF, reg_num & 0xFF]
        cmd = self.create_cmd(cmdbin)
        response = self.send_cmd(cmd, read_response=True)
        return response

    def write_registers(self):
        """预留：10H/06H 通用写入接口，供日后拓展"""
        pass

    def get_encoder(self, id):
        """
        单独读取编码器值
        Modbus:
        地址 0x31
        返回 0~65535
        """

        try:
            resp = self.read_registers(
                id,
                0x0031,
                0x0001
            )

            if not resp:
                return 0


            if len(resp) >= 7:
                data_bytes = resp[3:5]

                encoder = struct.unpack(
                    '>H',
                    data_bytes
                )[0]

                return encoder


        except Exception as e:
            print(
                f"[编码器读取失败] ID={id}: {e}"
            )


        return 0

    # ├──── 系统状态接口 ────
    def get_system_state(self, id) -> MotorState:
        """读取系统状态参数 (指令 0x43 读取16个寄存器)"""
        state = MotorState()
        
        # ─── 核心修改 1：把物理读取用 try-except 包裹起来，抓出潜在崩溃 ───
        try:
            resp = self.read_registers(id, 0x0043, 0x10)
        except Exception as read_err:
            print(f"[💥串口通信严重崩溃] 电机 ID {id} 在调用 read_registers 时发生底层异常: {read_err}")
            return state

        if not resp:
            print(f"[⚠️驱动调试] 电机 ID {id} 返回了空数据(None/Timeout)。可能是多线程总线冲突或波特率不对。")
            return state
            
        
        if len(resp) >= 32:
            if len(resp) >= 37 and resp[1] in [0x03, 0x04]:
                data_bytes = resp[3:35]
            else:
                data_bytes = resp[:32]
                
            try:
                # Reg 3: 总线电压 (单位: mV)
                voltage_mv = struct.unpack('>H', data_bytes[6:8])[0]
                # Reg 4: 实时相电流 (单位: mA)
                current_ma = struct.unpack('>H', data_bytes[8:10])[0]
                # 单独读取真实16bit编码器
                encoder_val = self.get_encoder(id)
                # Reg 8 & 9: 实时转速符号与数值
                speed_sign = struct.unpack('>H', data_bytes[16:18])[0]
                speed_rpm = struct.unpack('>H', data_bytes[18:20])[0]
                # Reg 10 & 11: 实时位置角度符号与数值
                pos_sign = struct.unpack('>H', data_bytes[20:22])[0]
                pos_val = struct.unpack('>H', data_bytes[22:24])[0]
                # Reg 15: 回零状态标志
                homing_flag = struct.unpack('>H', data_bytes[30:32])[0]
                
                motor_flag = 0
                if len(data_bytes) >= 34:
                    motor_flag = struct.unpack('>H', data_bytes[32:34])[0]
                
                state.voltage = voltage_mv / 1000.0
                state.current = current_ma
                state.encoder = encoder_val
                state.speed = -speed_rpm if speed_sign != 0 else speed_rpm
                
                raw_angle = -pos_val if pos_sign != 0 else pos_val
                state.angle = raw_angle * 360.0 / 65536.0
                
                state.homing_state = homing_flag
                state.motor_state = motor_flag
                
            except Exception as parse_err:
                print(f"[💥解析崩溃] 系统状态数据解析出错: {parse_err}")

        return state
    def get_position(self, id):
        """便捷获取角度位置 (指令 0x36)"""
        resp = self.read_registers(id, 0x0036, 0x03)
        if resp and len(resp) >= 11:
            sign = struct.unpack('>H', resp[3:5])[0]
            pos_val = struct.unpack('>i', resp[5:9])[0] # 包含高低位拼接，这里做简单适配
            angle = pos_val * 360.0 / 65536.0
            return -angle if sign != 0 else angle
        return 0.0

    def get_speed(self, id):
        """便捷获取实时转速 (指令 0x35)"""
        resp = self.read_registers(id, 0x0035, 0x02)
        if resp and len(resp) >= 9:
            sign = struct.unpack('>H', resp[3:5])[0]
            speed = struct.unpack('>H', resp[5:7])[0]
            return -speed if sign != 0 else speed
        return 0.0

    def get_encoder(self, id):
        """便捷获取编码器值 (指令 0x31)"""
        resp = self.read_registers(id, 0x0031, 0x01)
        if resp and len(resp) >= 7:
            return struct.unpack('>H', resp[3:5])[0]
        return 0

    # ├──── 基础控制接口 ────
    def set_control_mode(self, id, mode, save=0):
        cmdbin = self.mode_switch.copy()
        cmdbin[0] = id
        cmdbin[8] = save
        cmdbin[9] = mode
        cmd = self.create_cmd(cmdbin)
        self.send_cmd(cmd)

    def set_enable_state(self, id, state, sync=0):
        cmdbin = self.enable_ctl.copy()
        cmdbin[0] = id
        cmdbin[8] = state
        cmdbin[9] = sync
        cmd = self.create_cmd(cmdbin)
        self.send_cmd(cmd)

    def zero_position(self, id):
        cmdbin = self.clear_pos.copy()
        cmdbin[0] = id
        cmd = self.create_cmd(cmdbin)
        self.send_cmd(cmd)

    # ├──── 速度控制 ────
    def set_spd(self, id, spd, dir=0):
        if spd > self.MAXRPM: spd = 255
        cmdbin = self.spd_ctl.copy()
        cmdbin[0] = id
        effective_dir = dir ^ self.MOTOR_DIR.get(id, 0)
        if effective_dir == 1:
            cmdbin[7] ^= 0x01
        cmdbin[8] = 0 
        cmdbin[9] = (int(spd) >> 8) & 0xFF
        cmdbin[10] = int(spd) & 0xFF
        cmdbin[11] = 0x01
        cmd = self.create_cmd(cmdbin)
        self.send_cmd(cmd)
        
    def car_speed(self, id, spd, dir=0):
        if spd > self.MAXRPM: spd = 255
        cmdbin = self.spd_ctl.copy()
        cmdbin[0] = id
        effective_dir = dir ^ self.MOTOR_DIR.get(id, 0)
        if effective_dir == 1:
            cmdbin[7] ^= 0x01
        cmdbin[8] = self.acc
        cmdbin[9] = (int(spd) >> 8) & 0xFF
        cmdbin[10] = int(spd) & 0xFF
        cmdbin[11] = 0x01
        cmd = self.create_cmd(cmdbin)
        self.send_cmd(cmd)

    # ├──── 同步控制 ────
    def trigger_sync_motion(self):
        cmdbin = self.sync_motion.copy()
        cmd = self.create_cmd(cmdbin)
        self.send_cmd(cmd)

    # ├──── 小车运动接口 ────
    def car_dash(self, spd): 
        for mid in self.MOTOR_DIR:
            self.set_spd(mid, spd, 0)
            time.sleep(0.1)
        self.trigger_sync_motion()

    def car_fw(self, spd): 
        for mid in self.MOTOR_DIR:
            self.car_speed(mid, spd, 0)
            time.sleep(0.1)
        self.trigger_sync_motion()

    def car_bw(self, spd):
        for mid in self.MOTOR_DIR:
            self.car_speed(mid, spd, dir=1)
            time.sleep(0.1)
        self.trigger_sync_motion()

    def car_turn_left(self, spd):
        left_motors = dict(list(self.MOTOR_DIR.items())[:2])
        right_motors = dict(list(self.MOTOR_DIR.items())[2:])
        for mid in left_motors:
            if self.turn != -1:
                self.set_spd(mid, spd * self.turn, 0)
            else :
                self.set_spd(mid, spd, 1)
            time.sleep(0.05)
        for mid in right_motors:
            self.set_spd(mid, spd, 0)
            time.sleep(0.05)
        self.trigger_sync_motion()
        
    def car_turn_right(self, spd):
        left_motors = dict(list(self.MOTOR_DIR.items())[:2])
        right_motors = dict(list(self.MOTOR_DIR.items())[2:])
        for mid in left_motors:
            self.set_spd(mid, spd, 0)
            time.sleep(0.05)
        for mid in right_motors:
            if self.turn != -1:
                self.set_spd(mid, spd * self.turn, 0)
            else :
                self.set_spd(mid, spd, 1)
            time.sleep(0.05)
        self.trigger_sync_motion()

    def car_accelerate(self, flag):
        for mid in self.MOTOR_DIR:
            self.car_speed(mid, self.spd, 0)
            time.sleep(0.1)
        self.trigger_sync_motion()

    def car_decelerate(self):
        for mid in self.MOTOR_DIR:
            self.car_speed(mid, 0, 0)
            time.sleep(0.1)
        self.trigger_sync_motion()

    def car_turn(self):
        self.turn = -1
        self.car_turn_right(self.maxspd)
        self.turn = 0.5

    def car_truningrate(self, flag):
        if self.turn == 0 and flag == -1:
            self.turn = round(self.turn + flag, 1)
            print("Attention: Changed to reverse rotation mode.")
        elif self.turn == -1 and flag == -1:
            print("Error! Truningrate cannot be less than -1.")
        elif self.turn == -1 and flag == 1:
            self.turn = round(self.turn + flag, 1)
            print("Attention: Changed to differential rotation mode.")
        else :
            self.turn = round(self.turn + flag * 0.1, 1)
            print(f"Now truning rate is {self.turn}.")

    # ├──── 高级接口 ────
    def switch_all_motors_mode(self, mode):
        for mid in self.MOTOR_DIR:
            self.set_control_mode(mid, mode, save=0)
            time.sleep(0.05)

    def enable_all_motors(self, state):
        for mid in self.MOTOR_DIR:
            self.set_enable_state(mid, state, sync=0)
            time.sleep(0.05)

    # ├──── 调试接口 ────
    def print_system_state(self, id):
        """控制台打印电机系统全维状态"""
        state = self.get_system_state(id)
        print(f"\n--- Motor [{id}] System State ---")
        print(f"Voltage : {state.voltage:.2f} V")
        print(f"Current : {state.current} mA")
        print(f"Speed   : {state.speed} rpm")
        print(f"Angle   : {state.angle:.2f} °")
        print(f"Encoder : {state.encoder}")
        print(f"State   : 0x{state.motor_state:04X}")
        print("--------------------------------")

    # ├──── Top Motor Interface ────
    def control_top_motor(self, spd, dir=0):
        """防缠绕/中心连杆电机：速度控制封装"""
        self.set_spd(self.TOP_MOTOR_ID, spd, dir)
        
    def stop_top_motor(self):
        """防缠绕/中心连杆电机：立即停止封装"""
        self.set_spd(self.TOP_MOTOR_ID, 0, 0)
        
    def get_top_motor_state(self) -> MotorState:
        """获取顶层电机实时状态反馈（带排查日志）"""
        state = self.get_system_state(self.TOP_MOTOR_ID)
        
        # 💡 增加驱动层底层数据打印
        print(
            f"[MOTOR RAW] ID={self.TOP_MOTOR_ID} | "
            f"angle={state.angle:.1f} | "
            f"encoder={state.encoder} | "
            f"speed={state.speed}"
        )
        return state
        
    def rotate_top_motor(self, target_angle, speed=30, acceleration=0x80, relative=False):
        """
        防缠绕/中心连杆电机：位置模式闭环控制接口
        :param target_angle: 目标角度（度），支持正负浮点数
        :param speed: 运转目标转速 (RPM)，默认 30
        :param acceleration: 加速度曲线 (0x00 - 0xFF)，默认 0x80
        :param relative: False 为绝对位置模式（面向绝对零位），True 为相对当前位置偏转
        """
        # 1. 根据 16 细分计算物理脉冲数 (3200 脉冲 = 360 度)
        pulses = int(target_angle * 3200.0 / 360.0)
        
        # 2. 拷贝类中预留的 0x00FD 位置控制 Modbus 字节模板
        cmdbin = self.setang.copy()
        
        # 3. 动态改写目标电机 ID (0x05)
        cmdbin[0] = self.TOP_MOTOR_ID
        
        # 4. 填充方向与加速度
        # 在绝对位置模式下，大头电机固件方向通常填 0 即可，由驱动器内部自动判断最短路径
        cmdbin[7] = 0x00 
        cmdbin[8] = acceleration
        
        # 5. 填充转速 RPM (2字节，高字节在前)
        cmdbin[9] = (int(speed) >> 8) & 0xFF
        cmdbin[10] = int(speed) & 0xFF
        
        # 6. 核心：将脉冲数打包为 4 字节大端有符号整型 (int32)，完美兼容正负行程
        pulse_bytes = struct.pack('>i', pulses)
        cmdbin[11] = pulse_bytes[0]
        cmdbin[12] = pulse_bytes[1]
        cmdbin[13] = pulse_bytes[2]
        cmdbin[14] = pulse_bytes[3]
        
        # 7. 绝对/相对模式选择 (0x00: 绝对位置, 0x01: 相对位置)
        cmdbin[15] = 0x01 if relative else 0x00
        
        # 8. 多机同步标志 (0x00: 立即独立执行, 0x01: 等待同步总线)
        # 顶层电机不需要和底盘轮子同步对齐，设为 0x00 立即响应运转
        cmdbin[16] = 0x00
        
        # 9. 生成带 CRC16 校验码的完整报文，并推入异步串口线程任务队列
        cmd = self.create_cmd(cmdbin)
        self.add_task(lambda: self.send_cmd(cmd))
        print(f"[驱动] 顶层电机指令已发送: {'相对' if relative else '绝对'}目标角度 {target_angle}° ({pulses} 脉冲)")

    # ├──── 命令解析 ────
    def car_control_cmd_set(self, cmd: str) -> None :
        cmd = cmd.strip().lower()
        if cmd == "fw":
            self.add_task(lambda: self.car_fw(self.spd))
        elif cmd == "bw":
            self.add_task(lambda: self.car_bw(self.spd))
        elif cmd == "left":
            self.add_task(lambda: self.car_turn_left(self.spd))
        elif cmd == "right":
            self.add_task(lambda: self.car_turn_right(self.spd))
        elif cmd == "stop":
            self.add_task(lambda: self.car_decelerate()) 
        elif cmd == "turn":
            self.add_task(lambda: self.car_turn())
        elif cmd == "go":
            self.add_task(lambda: self.car_dash(self.maxspd))
        elif cmd == "skid":
            self.add_task(lambda: self.send_cmd(self.create_cmd(self.stop)))
        elif cmd == "openloop":
            self.add_task(lambda: self.switch_all_motors_mode(1)) 
        elif cmd == "closedloop":
            self.add_task(lambda: self.switch_all_motors_mode(2)) 
        elif cmd == "enable":
            self.add_task(lambda: self.enable_all_motors(1))
        elif cmd == "disable":
            self.add_task(lambda: self.enable_all_motors(0))
        elif cmd == "clearpos":
            self.add_task(lambda: [self.zero_position(mid) for mid in self.MOTOR_DIR])
            print("[系统] 已发送全车位置清零指令")
        else:
            print(f"[警告] 未识别的控制指令：'{cmd}'")
        time.sleep(0.2)

    def car_params_set(self, cmd):
        if cmd == 'SPD+':
            self.spd = min(self.spd + 1, self.MAXRPM)
            print(f"Now the speed is {self.spd}")
        elif cmd == 'SPD-':
            self.spd = max(self.spd - 1, 0)
        elif cmd == 'TURN+':
            self.add_task(lambda: self.car_truningrate(1))
        elif cmd == 'TURN-':
            self.add_task(lambda: self.car_truningrate(-1))
        elif cmd == 'ACC+':
            if self.acc == 0:
                print("Changed to accelerated startup mode.")
            self.acc = min(self.acc + 5, 255)
            print(f"Now the accelerate is {self.acc}")
        elif cmd == 'ACC-':
            self.acc = max(self.acc - 5, 0)
            print(f"Now the accelerate is {self.acc}")
            if self.acc == 0:
                print("Changed to constant speed startup mode.")
        else:
            print(f"[警告] 未识别的参数设置指令：{cmd}")

    # ├──── 队列与任务引擎 ────
    def add_task(self, task):
        if not self.task_queue.full():
            self.task_queue.put(task)
        else:
            print("命令队列已满，无法添加新命令")

    def process_task(self):
        while not self.ctl_stop_event.is_set():
            try:
                task = self.task_queue.get(timeout=0.1)
                if callable(task):
                    task()
                else:
                    print("[警告] 非法任务，跳过")
            except queue.Empty:
                continue

    def clear_cmd_queue(self):
        if hasattr(self, 'task_queue'):
            with self.task_queue.mutex:
                self.task_queue.queue.clear()
            print("[系统] 已清空命令队列")

    # ├──── 资源释放 ────
    def close_uart(self):
        self.ctl_stop_event.set()
        cmd = self.create_cmd(self.stop)
        self.send_cmd(cmd)
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()