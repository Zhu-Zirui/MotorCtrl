# exp1: Motor car control
import os
import importlib.util as implib
import pygame

device_name = 'USB-SERIAL CH340'
#CAR_COM = "com5" # 串口号,用于控制小车电机，查看设备管理器中的串口号设置
BAUD = 115200 # 默认波特率，无需改动

def loop(mctl):
    

    try:
        running = True

        # 主循环
        while running:

            
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:  # 如果用户点击了关闭按钮

                    running = False
                elif event.type == pygame.KEYDOWN:  # 检测按键按下事件

                        if event.key == pygame.K_ESCAPE:  # 如果按下的是 ESC 键
                            running = False
                            print("ESC按下,程序退出")
                        
                   
                        elif event.unicode == ' ':  # 如果按下的键是空格键
                            mctl.car_control_cmd_set('STOP')
                            print("空格键被按下")
                        elif event.unicode == 'w':  # 如果按下的键是W键
                            mctl.car_control_cmd_set('FW')
                            print("W键被按下")
                        elif event.unicode == 's':  # 如果按下的键是S键
                            mctl.car_control_cmd_set('BW')
                            print("S键被按下")
                        elif event.unicode == 'a':  # 如果按下的键是A键
                            mctl.car_control_cmd_set('TL')
                            print("A键被按下")
                            
                        elif event.unicode == 'd':  # 如果按下的键是D键
                            mctl.car_control_cmd_set('TR')
                            print("D键被按下")
                           
                            
                        elif event.unicode == 'x':  # 如果按下的键是X键
                            mctl.car_params_set('SPD+')
                            print("car_spd = ", mctl.spd)
                        elif event.unicode == 'z':  # 如果按下的键是Z键
                            mctl.car_params_set('SPD-')
                            print("car_spd = ", mctl.spd)
                        elif event.unicode == 'q':  # 如果按下的键是Q键
                            mctl.car_params_set('TURN-')
                            print("car_turn_angle = ", mctl.angle)
                        elif event.unicode == 'e':  # 如果按下的键是E键
                            mctl.car_params_set('TURN+')
                            print("car_turn_angle = ", mctl.angle)
                        elif event.unicode == 'c':  # 如果按下的键是c键
                            mctl.car_params_set('ACC-')
                            print("car_accel = ", mctl.acc)
                        elif event.unicode == 'v':  # 如果按下的键是V键
                            mctl.car_params_set('ACC+')
                            print("car_accel = ", mctl.acc)   
                        else:
                            pass

            pygame.display.flip()
    except KeyboardInterrupt:
        print("程序被中断")
    finally:
        mctl.close_uart()
        print("串口已关闭")

if __name__=='__main__':

    # 获取当前运行文件的绝对路径
    current_file_path = os.path.abspath(__file__)
    file_directory = os.path.dirname(current_file_path)
    print("当前运行文件所在的目录路径:", file_directory)
    
    from AllMotorCtrl import Motor_ctl
    print("已改用 AllMotorCtrl.Motor_ctl 作为底层驱动")


    """ 小车控制端串口配置 """
    mctl = Motor_ctl()
    CAR_COM = mctl.find_port_by_name(device_name)
    if CAR_COM is None:
        print("未找到小车控制端串口")
        exit(-1)
    print("小车控制端串口:", CAR_COM)
    mctl.init_motor(CAR_COM, BAUD) 
    
    if mctl.ser is None:
        print("小车控制端串口初始化失败！")
        exit(-1)
    pygame.init()
    #打开pygame窗口
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("小车控制端")
    pygame.display.flip()
    print("请在pygame窗口中输入w,s,a,d来控制小车的前进、后退、左转和右转,输入空格键停止,输入q,e来调整转向角度,输入c,v来调整加速度")
    #在pygame窗口中显示控制提示
    font = pygame.font.Font(None, 30)
    text = font.render("input: w,s,a,d for car forword, backword, turn lest and right!",True, (255, 255, 255))
    screen.blit(text, (10, 10))
    text = font.render("input: space for stop, q, e for adjust turn angle,",True, (255, 255, 255))
    screen.blit(text, (10, 50))
    text = font.render("input: c, v for adjust speed!",True, (255, 255, 255))
    screen.blit(text, (10, 90))
    
    loop(mctl)
    #关闭pygame窗口

    pygame.quit()