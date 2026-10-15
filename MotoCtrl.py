import serial
import serial.tools.list_ports
import struct
import time

#注意，以下是单机指令，多机指令请自行修改
spd_ctl = [0x01, 0x10, 0x00, 0xF6, 0x00, 0x03, 0x06, 0x00, 0x0a, 0x00, 0x80, 0x00, 0x00, 0x9B, 0x25]
stop = [0x00, 0x10, 0x00, 0xFE, 0x00, 0x01, 0x02, 0x98, 0x00, 0xD5, 0xDE] #广播停止
openCTLMOD = [0x00, 0x10, 0x00, 0x46, 0x00, 0x02, 0x04, 0x69, 0x00, 0x01, 0x00, 0x6F, 0x45] #广播 开控制模式，不储存
closeCTLMOD = [0x00, 0x10, 0x00, 0x46, 0x00, 0x02, 0x04, 0x69, 0x00, 0x02, 0x00, 0x6F, 0xB5] #广播 闭环控制模式，不储存
spd_Z = [0x00, 0x10, 0x00, 0xF6, 0x00, 0x03, 0x06, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x41, 0xC8] #多机速度归零 大加速度

ang2zero = [0x01, 0x06, 0x00, 0x0A, 0x00, 0x01, 0x69, 0x8C] #05 06 00 0A 00 01 69 8C角度位置清零
readencoderval = [0x01, 0x04, 0x00, 0x31, 0x00, 0x01, 0x61, 0x81] # 05 04 00 31 00 01 61 81 读取编码器数值 返回 01H 04H 02H 编码器值(0-65535) CRC16
final_ang = [0x01, 0x04, 0x00, 0x33, 0x00, 0x03, 0x41, 0x80] #05 04 00 33 00 03 41 80 读取电机设定的最终目标位置, 返回01H 04H 06H 符号(0 正1 负) 电机设定的最终目标位置(0-65535) CRC16,\
                                                                #*转换成角度公式= 电机设定的最终目标位置* 360 / 65536 #采用的减速齿轮3:1
current_ang = [0x01, 0x04, 0x00, 0x36, 0x00, 0x03, 0x51, 0x81] #05 04 00 36 00 03 51 81 读取电机当前实时的目标位置, 返回01H 04H 06H 符号(0 正1 负) 电机当前位置(0-65535) CRC16
                                                                #*转换成角度公式= 电机实时位置* 360 / 65536 #采用的减速齿轮3:1
readparam = [0x01, 0x04, 0x00, 0x42, 0x00, 0x0f, 0x11, 0x9e] #读取驱动参数，返回05 04 1E |21 15|        19 02|02 02|00 10|01 00|03 E8|09 60|13 88|05 07|05 03| 00 01|00 08|08 98|07 D0|00 03 |D4 60 
                                                                                #05 04 1E  字节数 变量数   REG2  REG3  REG4  REG5  REG6  REG7  REG8  REG9  REG10 REG11 REG12 REG13 REG14 REG15   CRC
                                                                                # REG4 Lo:10表示16细分，16个脉冲走1.8°，REG2Hi：表示为1.8°步进，200步进走一周360° 当前为3200脉冲=360°
#writeparam = [0x01, 0x10, 0x00, 0x48, 0x00, 0x0f, 0x1e, 19 02|02 02|00 10|01 00|03 E8|09 60|13 88|05 07|05 03| 00 01|00 08|08 98|07 D0|00 03 |D4 60 |CRC16]
setang = [0x01, 0x10, 0x00, 0xFD, 0x00, 0x05, 0x0A, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0c, 0x80, 0x00, 0x00, 0xCD, 0xBB] # 05 10 00 FD 00 05 0A 00 80 00 10 00 00 0c 80 00 00 CD BB 角度转动控制
            #            保留                         | 方向 加速度  速度(rpm) |     脉冲数 3200*3=360°  |相对  多机 |CRC16

#ser = None


device_name = 'USB-SERIAL CH340'

def find_port_by_name(device_name):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if device_name.lower() in port.description.lower():
            return port.device
    return None





def open_serial():
    """初始化并打开串口"""
    port = find_port_by_name(device_name)
    if port:
        print(f"找到设备 '{device_name}' 的端口号: {port}")
        try:
            ser = serial.Serial(port, 115200, timeout = 1)  # 打开串口，波特率为115200 如果1秒内没有读取到完整的一行数据
            print(f"串口 {port} 已打开")
            return ser
        except serial.SerialException as e:
            print(f"打开串口失败: {e}")
            return None
    else:
        print(f"未找到设备 '{device_name}' 的端口号")
        return None

# CRC-16生成多项式

CRC16_POLYNOMIAL = 0xa001

# 计算CRC-16校验值
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF  # 初始值为0xFFFF

    for byte in data:
        crc ^= byte  # 将数据字节与CRC值异或

        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ CRC16_POLYNOMIAL) & 0xFFFF  # 如果最低位为1，进行位移和异或
            else:
                crc >>= 1  # 否则只进行位移

    return crc & 0xFFFF  # 确保结果是16位
def recal_crc16(data: list) -> list:
    if len(data) < 2:
        raise ValueError("Data must have at least 2 elements to store the CRC16 value.")
    
    data_short = bytes(data[:-2])  # 转换为字节串（假设data中的元素是整数且在0-255范围内）
    crc = calculate_crc16(data_short)
    data[-2] = crc & 0xFF  # 低8位
    data[-1] = (crc >> 8) & 0xFF  # 高8位
    return data

def create_cmd(data):
    data_short = bytes(data[:-2])  # 去除最后两个元素并转换为字节串
    #data_short = data[:-2]
    crc = calculate_crc16(data_short)
    crc_bytes = struct.pack('<H', crc)  # 将CRC值打包为两个字节的小端序格式

    return data_short + crc_bytes
def send_cmd(ser, bytes_data):
    #hex_data = ' '.join('{:02x}'.format(b) for b in bytes_data)
    try:
        #print("LEN: ",len(bytes_data),"CMD: ",hex_data)

        ser.write(bytes_data)

        hex_data = ' '.join('{:02x}'.format(b) for b in bytes_data)
        print("send_cmd: %s" % hex_data)
        return True #hex_data
    except Exception as e:
        print("Failed to write data : %s" % bytes,"with error: ",e)   

def readuart(ser):
    raw_data = b''
    delayt=10
    
    #print("readuart-delay: %s ms" % delayt)
    while True: # 循环直到没有数据可读

        time.sleep(delayt/1000)
    #    response = ser.read(ser.in_waiting)
    #    hex_data = ' '.join('{:02x}'.format(b) for b in response)
    #    print("response: %s" % hex_data)
        if ser.in_waiting == 0:
            #print("readuart-no data")
            return raw_data
            #break
        try:
            # 读取一行数据（保持为字节格式）
            data = ser.readline()
            #print("readuart-data: %s" % data)
            raw_data = raw_data + data 

            # 将字节数据转换为HEX字符串
            # 方法1：使用空格分隔的HEX表示
            #hex_data = ' '.join('{:02x}'.format(b) for b in raw_data)
            
            #print(hex_data)
            
        except Exception as e:
            print("Failed to read data !") 
            return None
    
    
def combine_bytes_to_32bit_int(byte_list, start_index, end_index):
    """
    将byte_list中从start_index到end_index（包含）的字节按从高位到低位的顺序合成为一个32位整数。
    
    :param byte_list: 包含字节的列表或数组
    :param start_index: 开始索引（包含）
    :param end_index: 结束索引（包含）
    :return: 合成后的32位整数
    """
    if len(byte_list) < end_index + 1:
        raise IndexError("byte_list长度不足以包含从{}到{}的索引".format(start_index, end_index))
    
    result = 0
    for i in range(start_index, end_index + 1):
        #print(i,byte_list[i])
        # 将当前字节左移相应的位数，然后与result进行或操作
        # 注意，因为是从高位到低位，所以每次左移的位数是递减的
        result = (result << 8) | byte_list[i]
    
    return result

def decompose_32bit_int_to_bytes( num):
    """
    将32位的整数num按从高位到低位的顺序分解为字节数组。
    
    :param num: 32位的整数
    :return: 包含四个字节的列表
    """
    # 将整数转换为无符号的32位整数（如果需要的话），以避免符号扩展
    # Python 3中int是任意精度的，但这里我们明确为32位
    # 注意：在Python中通常不需要这样做，因为Python的整数会自动处理，但为了清晰性，可以加上
    # 如果要严格限制为32位，并且关心性能，则可能需要使用特定的库（如ctypes或numpy）
    # 但为了简单起见，这里我们直接使用Python的int
    
    # 将整数分解为四个字节
    num = int(num)  # 将整数转换为无符号的32位整数  
    byte_3 = (num >> 24) & 0xFF  # 提取最高位字节
    byte_2 = (num >> 16) & 0xFF  # 提取次高位字节
    byte_1 = (num >> 8) & 0xFF   # 提取第三位字节
    byte_0 = num & 0xFF          # 提取最低位字节
    
    # 返回字节列表
    return [byte_3, byte_2, byte_1, byte_0]
def get_angle(ser):
    cmd = create_cmd(current_ang)
    send_cmd(ser, cmd)
    cmd = None
    time.sleep(0.5)
    raw_data = readuart(ser)
    if len(raw_data) > 1 :
        #print("raw_data:",raw_data)
        hex_data = ' '.join('{:02x}'.format(b) for b in raw_data)
        print("Hex_data:",hex_data)
        val = combine_bytes_to_32bit_int(raw_data,5,8)
        #print ("raw_val[5 - 8]: ",val)
        res_val = (float(val)*360.0/65536.0)
        return res_val
    else:
        return None
def set_angle(ser, angle, dir=0 ):
    pulses = angle*200*16/360
    pulsebytes = decompose_32bit_int_to_bytes(pulses)
    cmdbyte = setang[:]
    cmdbyte[11:15] = pulsebytes
    
    if dir != 0:
        cmdbyte[7] = 0x01
    cmd = create_cmd(cmdbyte)
    send_cmd(ser, cmd)
def set_spd(ser, spd, dir=0):
    if spd > 255:
        spd = 255

    cmdbin = spd_ctl # 复制数组
    byte_H = (int(spd) >> 8) & 0xFF
    byte_L = int(spd) & 0xFF

    cmdbin[9] = byte_H
    cmdbin[10] = byte_L
    #cmdbin[8] = 10 # 设置固定加速度值

    if dir == 1:
        cmdbin[7] ^= 0x01  # Toggle bit 7
    #print(cmdbin)
    cmd = create_cmd(cmdbin)

    send_cmd(ser,cmd)
    
def main():
    ser = open_serial()

    if not ser:
        return
    
    try:
        while True:
            cmd = None
            command = input("请输入要发送的命令(help查看帮助): ")
            if command.lower() == 'exit':  # 输入'exit'退出程序
                break
            elif command.lower() == 'run':
                command = input("请设置的转速(0~255): ")
                if command.isdigit():
                    spd = int(command)
                else :
                    continue
                command = input("请输入方向(缺省 0:正向, 1:反向): ")
                if command.lower() == '1':
                    dir = 1
                else :
                    dir = 0
                set_spd(ser, spd=spd, dir=dir)
            elif command.lower() == 'openctrl':
                cmd = create_cmd(openCTLMOD)
            elif command.lower() == 'closectrl':
                cmd = create_cmd(closeCTLMOD)
            elif command.lower() == 'spd2zero':
                cmd = create_cmd(spd_Z)
            elif command.lower() == 'ang2zero':
                cmd = create_cmd(ang2zero)
            #elif command.lower() == 'readencoderval':
            #   cmd = create_cmd(readencoderval)
            #elif command.lower() == 'final_ang':
            #    cmd = create_cmd(final_ang)
            elif command.lower() == 'current_ang':
                res = get_angle(ser)
                if res is not None:
                    print("当前角度位置: %.2f°" % res)
            elif command.lower() == 'set_ang':
                command = input("请输入要改变的角度: ")
                if command.isdigit():
                    angle = float(command)
                else :
                    continue
                command = input("请输入方向(缺省 0:正向, 1:反向): ")
                if command.lower() == '1':
                    dir = 1
                else :
                    dir = 0

                set_angle(ser, angle=angle,dir=dir)
            
            elif command.lower() == 'readparam':
                cmd = create_cmd(readparam)
            elif command.lower() == 'stop':
                cmd = create_cmd(stop)
            elif command.lower() == 'setang':
                cmd = create_cmd(setang)
                
            elif command.lower() == 'help':
                print("可用命令如下：")
                print("run: 运行")
                print("stop: 停止")
                print("spd2zero: 速度归零")
                print("openctrl: 开启控制模式")
                print("closectrl: 关闭控制模式")
                print("ang2zero: 角度位置清零")
                #print("readencoderval: 读取编码器值")
                #print("final_ang: 读取最终目标位置")
                print("current_ang: 读取当前位置")
                print("readparam: 读取参数")
                #print("writeparam: 写入参数")
                print("setang: 设置角度")
                print("exit: 退出程序")
            else:
                print("未知命令！")
            
            if cmd is not None:
                send_cmd(ser, cmd)
            
            raw_data = readuart(ser)
            if len(raw_data) > 1 :
                hex_data = ' '.join('{:02x}'.format(b) for b in raw_data)
                print("response: %s" % hex_data)
           
    except KeyboardInterrupt:
        print("程序被中断")
    finally:
        ser.close()
        print("串口已关闭")

if __name__ == "__main__":
    main()
