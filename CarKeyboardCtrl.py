# CarKeyboardCtrl.py
# 四电机小车键盘控制（纯终端版，无需 pygame 窗口）
# 依赖 AllMotorCtrl.Motor_ctl（同一套 Modbus-RTU 协议，含右两侧电机方向补偿）
#
# 电机布局（与 Motor_ctl.MOTOR_DIR 一致）：
#   1=左前  3=左后 | 2=右前  4=右后   （2/4 已在 Motor_ctl 内做方向补偿）
#
# 键位说明：
#   按住 w 前进 / s 后退 / a 左转 / d 右转（松开自动平滑停止）
#   单独按 a/d：以一半基础速度做弧线差速转弯；Shift+a/d：原地旋转
#   空格   立即停止
#   x / z  基础速度 +/-      q / e  转向强度 +/-（1.0 为最急转弯）
#   ESC    退出
#
# 注意：使用 Windows API 全局键盘状态，即使终端失焦按键依然生效，
#       调试时请勿在其他窗口打字，避免小车意外动作！

import ctypes
import time
from AllMotorCtrl import Motor_ctl

DEVICE_NAME = 'USB-SERIAL CH340'
BAUD = 115200

# 速度安全限幅（Motor_ctl.set_spd 内部同样会限幅）
MAX_RPM = 255

# ── 转弯（差速）参数，想调弯的急缓改这里 ──────────────────
SOLO_TURN_SPEED_RATIO = 0.5   # 单独按 a/d 转弯时的前进速度 = spd × 此系数
                              #   调小 → 弯更急（0 时接近原地转）
                              #   调大 → 弯更缓（转弯半径更大）
DEFAULT_TURN_RATIO = 0.3      # 转向强度初始值（差速量 w = spd × turn_ratio）
                              #   范围 0.1~1.0，1.0 最急（内侧轮反转）
                              #   运行中随时按 q / e 实时增减
# ─────────────────────────────────────────────────────────

# 虚拟键码（Windows Virtual-Key）
_VK = {
    'w': 0x57, 's': 0x53, 'a': 0x41, 'd': 0x44,
    ' ': 0x20, 'x': 0x58, 'z': 0x5A, 'q': 0x51, 'e': 0x45,
    'shift': 0x10,
    'esc': 0x1B,
}


class Keyboard:
    """Windows 控制台键盘轮询：GetAsyncKeyState，无需窗口和焦点"""

    def __init__(self):
        self.user32 = ctypes.windll.user32
        self.prev = {k: False for k in _VK}

    def _down(self, key):
        return bool(self.user32.GetAsyncKeyState(_VK[key]) & 0x8000)

    def held(self, *keys):
        """当前是否按住（任意一个）"""
        return any(self._down(k) for k in keys)

    def pressed(self, key):
        """按键上升沿（刚按下瞬间返回 True，用于触发型命令）"""
        now = self._down(key)
        was = self.prev[key]
        self.prev[key] = now
        return now and not was


class CarKeyboard:
    """键盘控制层：差速运动学 + 每帧限步平滑（软件加减速）"""

    LEFT_IDS = [1, 3]    # 左前 / 左后
    RIGHT_IDS = [2, 4]   # 右前 / 右后

    def __init__(self, mctl, spd=20, turn_ratio=0.5, accel_step=4):
        """
        :param mctl: Motor_ctl 实例
        :param spd: 基础线速度 RPM
        :param turn_ratio: 转向强度 0~1，1.0 时按住转向键即原地转
        :param accel_step: 每帧轮速最大变化量 RPM，越小加减速越平滑
        """
        self.m = mctl
        self.spd = float(spd)
        self.turn_ratio = turn_ratio
        self.accel_step = float(accel_step)

        # 当前平滑速度 与 目标速度（RPM）
        self.v = 0.0
        self.w = 0.0        # w>0 表示右转（左轮快、右轮慢）
        self.v_t = 0.0
        self.w_t = 0.0

    # ---------- 目标设置 ----------
    def set_motion(self, v, w):
        """设置目标速度：v 线速度，w 差速量（w>0 右转）"""
        self.v_t = max(-MAX_RPM, min(MAX_RPM, float(v)))
        self.w_t = max(-MAX_RPM, min(MAX_RPM, float(w)))

    def stop_now(self):
        """紧急停止：清零目标并广播停止"""
        self.v_t = self.w_t = self.v = self.w = 0.0
        self.m.send_cmd(self.m.create_cmd(self.m.stop))

    # ---------- 平滑与下发 ----------
    def _ramp(self, cur, target):
        d = target - cur
        if abs(d) <= self.accel_step:
            return target
        return cur + self.accel_step if d > 0 else cur - self.accel_step

    def update(self):
        """主循环每帧调用：向目标速度平滑靠拢，若轮速有变化则下发四轮指令"""
        self.v = self._ramp(self.v, self.v_t)
        self.w = self._ramp(self.w, self.w_t)

        # 差速运动学：w>0 右转（左轮加速、右轮减速），v=0 时即为原地转
        left = self.v + self.w
        right = self.v - self.w
        left = max(-MAX_RPM, min(MAX_RPM, left))
        right = max(-MAX_RPM, min(MAX_RPM, right))

        self._apply(left, right)

    def _apply(self, left, right):
        """把带符号的左右轮速写到四个电机并同步触发"""
        m = self.m
        for mid in self.LEFT_IDS:
            m.set_spd(mid, abs(left), 0 if left >= 0 else 1)
        for mid in self.RIGHT_IDS:
            m.set_spd(mid, abs(right), 0 if right >= 0 else 1)
        m.trigger_sync_motion()


def main():
    mctl = Motor_ctl()
    port = mctl.find_port_by_name(DEVICE_NAME)
    if port is None:
        print(f"未找到 '{DEVICE_NAME}' 串口，请检查连接")
        return
    print("小车串口:", port)
    mctl.init_motor(port, BAUD)
    if mctl.ser is None:
        print("串口初始化失败")
        return

    # 使能四个电机并等待稳定
    mctl.enable_all_motors(1)
    time.sleep(0.5)

    car = CarKeyboard(mctl, spd=20, turn_ratio=DEFAULT_TURN_RATIO, accel_step=4)
    kb = Keyboard()

    print("=" * 50)
    print("四电机小车终端控制（无需窗口焦点）")
    print("  按住 w/s/a/d  前进/后退/弧线左转/弧线右转，松开平滑停车")
    print("  Shift+ad 原地旋转   空格 急停   x/z 速度+/-   q/e 转向强度+/-   ESC 退出")
    print("  提示：键盘为全局监听，调试时别在其他窗口打字！")
    print("=" * 50)

    try:
        while True:
            # 触发型按键（上升沿）
            if kb.pressed('esc'):
                break
            if kb.pressed(' '):
                car.stop_now()
                print("\n[急停]")
            if kb.pressed('x'):
                car.spd = min(MAX_RPM, car.spd + 10)
            if kb.pressed('z'):
                car.spd = max(0, car.spd - 10)
            if kb.pressed('e'):
                car.turn_ratio = min(1.0, round(car.turn_ratio + 0.1, 1))
            if kb.pressed('q'):
                car.turn_ratio = max(0.1, round(car.turn_ratio - 0.1, 1))

            # 按住型：按住即动，松开自动回零平滑停止
            v = car.spd * ((1 if kb.held('w') else 0) - (1 if kb.held('s') else 0))
            w = car.spd * car.turn_ratio * ((1 if kb.held('d') else 0) - (1 if kb.held('a') else 0))
            # 单独按转向键时：默认做弧线差速转弯（一半基础速度）；Shift+转向键 = 原地旋转
            if w != 0 and v == 0:
                v = 0.0 if kb.held('shift') else car.spd * SOLO_TURN_SPEED_RATIO
            car.set_motion(v, w)
            car.update()

            # 状态行刷新
            left = int(car.v + car.w)
            right = int(car.v - car.w)
            print(f"\r  spd:{car.spd:4.0f}  turn:{car.turn_ratio:.1f}  "
                  f"v:{car.v:5.0f}  L:{left:4d}  R:{right:4d}   ",
                  end="", flush=True)

            time.sleep(1 / 30)  # 30 FPS

    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        try:
            car.stop_now()
        except Exception:
            pass
        mctl.close_uart()
        print("\n已停止并关闭串口")


if __name__ == "__main__":
    main()
