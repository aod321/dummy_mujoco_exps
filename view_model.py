#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import argparse
import collections

# 创建一个用于存储传感器数据的类
class SensorDataCollector:
    def __init__(self, history_length=500):
        self.history_length = history_length
        
        # 初始化数据历史
        self.force_history = {
            'x': collections.deque([0] * history_length, maxlen=history_length),
            'y': collections.deque([0] * history_length, maxlen=history_length),
            'z': collections.deque([0] * history_length, maxlen=history_length)
        }
        
        self.torque_history = {
            'x': collections.deque([0] * history_length, maxlen=history_length),
            'y': collections.deque([0] * history_length, maxlen=history_length),
            'z': collections.deque([0] * history_length, maxlen=history_length)
        }
        
        # 存储当前值
        self.current_force = [0, 0, 0]
        self.current_torque = [0, 0, 0]
    
    def update(self, force_data, torque_data):
        # 更新当前值
        self.current_force = force_data
        self.current_torque = torque_data
        
        # 更新力数据历史
        self.force_history['x'].append(force_data[0])
        self.force_history['y'].append(force_data[1])
        self.force_history['z'].append(force_data[2])
        
        # 更新力矩数据历史
        self.torque_history['x'].append(torque_data[0])
        self.torque_history['y'].append(torque_data[1])
        self.torque_history['z'].append(torque_data[2])
    
    def get_current_values(self):
        return {
            'force': self.current_force,
            'torque': self.current_torque
        }
    
    # 获取最大和最小值，用于检测峰值
    def get_max_values(self):
        return {
            'force_x': max(self.force_history['x']),
            'force_y': max(self.force_history['y']),
            'force_z': max(self.force_history['z']),
            'torque_x': max(self.torque_history['x']),
            'torque_y': max(self.torque_history['y']),
            'torque_z': max(self.torque_history['z'])
        }
    
    def get_min_values(self):
        return {
            'force_x': min(self.force_history['x']),
            'force_y': min(self.force_history['y']),
            'force_z': min(self.force_history['z']),
            'torque_x': min(self.torque_history['x']),
            'torque_y': min(self.torque_history['y']),
            'torque_z': min(self.torque_history['z'])
        }
    
    def get_formatted_display(self, width=50):
        """生成一个简单的文本图表显示力和力矩数据"""
        display = "\033[2J\033[H"  # 清屏并将光标移动到左上角
        display += "=== 传感器数据实时监控 ===\n\n"
        
        # 显示当前值
        display += f"力传感器 (N)    : X: {self.current_force[0]:+8.3f}  Y: {self.current_force[1]:+8.3f}  Z: {self.current_force[2]:+8.3f}\n"
        display += f"力矩传感器 (N·m): X: {self.current_torque[0]:+8.3f}  Y: {self.current_torque[1]:+8.3f}  Z: {self.current_torque[2]:+8.3f}\n\n"
        
        # 显示最大和最小值
        max_vals = self.get_max_values()
        min_vals = self.get_min_values()
        
        display += "--- 数据范围 ---\n"
        display += f"力 X: [{min_vals['force_x']:+6.2f} to {max_vals['force_x']:+6.2f}] N    "
        display += f"力矩 X: [{min_vals['torque_x']:+6.2f} to {max_vals['torque_x']:+6.2f}] N·m\n"
        
        display += f"力 Y: [{min_vals['force_y']:+6.2f} to {max_vals['force_y']:+6.2f}] N    "
        display += f"力矩 Y: [{min_vals['torque_y']:+6.2f} to {max_vals['torque_y']:+6.2f}] N·m\n"
        
        display += f"力 Z: [{min_vals['force_z']:+6.2f} to {max_vals['force_z']:+6.2f}] N    "
        display += f"力矩 Z: [{min_vals['torque_z']:+6.2f} to {max_vals['torque_z']:+6.2f}] N·m\n\n"
        
        # 为力的X分量创建条形图
        force_x = self.current_force[0]
        max_force_x = max(10, abs(max_vals['force_x']), abs(min_vals['force_x']))
        force_x_bar_width = int((force_x / max_force_x) * (width // 2))
        
        display += "--- 力 X 分量条形图 ---\n"
        display += f"{-max_force_x:+6.1f} "
        if force_x < 0:
            display += " " * (width // 2 + force_x_bar_width) + "|" + "#" * abs(force_x_bar_width) + " "
        else:
            display += " " * (width // 2) + "|" + "#" * force_x_bar_width + " "
        display += f" {max_force_x:+6.1f}\n"
        display += " " * 7 + "-" * width + "\n"
        display += " " * (width // 2 + 6) + "0\n\n"
        
        # 为力的Y分量创建条形图
        force_y = self.current_force[1]
        max_force_y = max(10, abs(max_vals['force_y']), abs(min_vals['force_y']))
        force_y_bar_width = int((force_y / max_force_y) * (width // 2))
        
        display += "--- 力 Y 分量条形图 ---\n"
        display += f"{-max_force_y:+6.1f} "
        if force_y < 0:
            display += " " * (width // 2 + force_y_bar_width) + "|" + "#" * abs(force_y_bar_width) + " "
        else:
            display += " " * (width // 2) + "|" + "#" * force_y_bar_width + " "
        display += f" {max_force_y:+6.1f}\n"
        display += " " * 7 + "-" * width + "\n"
        display += " " * (width // 2 + 6) + "0\n\n"
        
        # 为力的Z分量创建条形图
        force_z = self.current_force[2]
        max_force_z = max(10, abs(max_vals['force_z']), abs(min_vals['force_z']))
        force_z_bar_width = int((force_z / max_force_z) * (width // 2))
        
        display += "--- 力 Z 分量条形图 ---\n"
        display += f"{-max_force_z:+6.1f} "
        if force_z < 0:
            display += " " * (width // 2 + force_z_bar_width) + "|" + "#" * abs(force_z_bar_width) + " "
        else:
            display += " " * (width // 2) + "|" + "#" * force_z_bar_width + " "
        display += f" {max_force_z:+6.1f}\n"
        display += " " * 7 + "-" * width + "\n"
        display += " " * (width // 2 + 6) + "0\n\n"
        
        # 为力矩的X分量创建条形图
        torque_x = self.current_torque[0]
        max_torque_x = max(2, abs(max_vals['torque_x']), abs(min_vals['torque_x']))
        torque_x_bar_width = int((torque_x / max_torque_x) * (width // 2))
        
        display += "--- 力矩 X 分量条形图 ---\n"
        display += f"{-max_torque_x:+6.1f} "
        if torque_x < 0:
            display += " " * (width // 2 + torque_x_bar_width) + "|" + "#" * abs(torque_x_bar_width) + " "
        else:
            display += " " * (width // 2) + "|" + "#" * torque_x_bar_width + " "
        display += f" {max_torque_x:+6.1f}\n"
        display += " " * 7 + "-" * width + "\n"
        display += " " * (width // 2 + 6) + "0\n\n"
        
        # 为力矩的Y分量创建条形图
        torque_y = self.current_torque[1]
        max_torque_y = max(2, abs(max_vals['torque_y']), abs(min_vals['torque_y']))
        torque_y_bar_width = int((torque_y / max_torque_y) * (width // 2))
        
        display += "--- 力矩 Y 分量条形图 ---\n"
        display += f"{-max_torque_y:+6.1f} "
        if torque_y < 0:
            display += " " * (width // 2 + torque_y_bar_width) + "|" + "#" * abs(torque_y_bar_width) + " "
        else:
            display += " " * (width // 2) + "|" + "#" * torque_y_bar_width + " "
        display += f" {max_torque_y:+6.1f}\n"
        display += " " * 7 + "-" * width + "\n"
        display += " " * (width // 2 + 6) + "0\n\n"
        
        # 为力矩的Z分量创建条形图
        torque_z = self.current_torque[2]
        max_torque_z = max(2, abs(max_vals['torque_z']), abs(min_vals['torque_z']))
        torque_z_bar_width = int((torque_z / max_torque_z) * (width // 2))
        
        display += "--- 力矩 Z 分量条形图 ---\n"
        display += f"{-max_torque_z:+6.1f} "
        if torque_z < 0:
            display += " " * (width // 2 + torque_z_bar_width) + "|" + "#" * abs(torque_z_bar_width) + " "
        else:
            display += " " * (width // 2) + "|" + "#" * torque_z_bar_width + " "
        display += f" {max_torque_z:+6.1f}\n"
        display += " " * 7 + "-" * width + "\n"
        display += " " * (width // 2 + 6) + "0\n"
        
        return display

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='MuJoCo 模型可视化工具')
    parser.add_argument('--model', type=str, default='mjmodel/bimanual_dummy_transfer_cube.xml',
                        help='模型文件路径 (默认: mjmodel/bimanual_dummy_transfer_cube.xml)')
    parser.add_argument('--duration', type=float, default=0,
                        help='模拟持续时间(秒)，0表示无限制 (默认: 0)')
    parser.add_argument('--plot', action='store_true', help='启用传感器数据文本可视化')
    parser.add_argument('--jacobian', action='store_true', help='启用基于雅可比矩阵的末端位姿控制')
    parser.add_argument('--site', type=str, default='left_gripper_site', 
                        help='用于雅可比控制的末端站点名称 (默认: left_gripper_site)')
    parser.add_argument('--delta', type=float, default=0.001,
                        help='末端位姿的增量大小 (默认: 0.001)')
    parser.add_argument('--compliance', action='store_true', help='启用基于力传感器的顺应控制')
    parser.add_argument('--force-gain', type=float, default=0.01,
                        help='力传感器增益系数 (默认: 0.01)')
    parser.add_argument('--force-threshold', type=float, default=0.5,
                        help='力传感器阈值，低于此值的力将被忽略 (默认: 0.5)')
    args = parser.parse_args()

    # 确保模型文件存在
    if not os.path.exists(args.model):
        print(f"错误: 模型文件 '{args.model}' 不存在")
        return

    # 加载模型
    print(f"加载模型: {args.model}")
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    # 初始化模型
    mujoco.mj_resetData(model, data)
    
    # 如果模型有关键帧，使用第一个关键帧初始化
    # if model.nkey > 0:
        # mujoco.mj_resetDataKeyframe(model, data, 0)
    
    # 打印传感器信息
    if model.nsensor > 0:
        print(f"\n模型包含 {model.nsensor} 个传感器:")
        for i in range(model.nsensor):
            # 获取传感器名称 - 使用 mj_id2name 函数
            sensor_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            sensor_type = mujoco.mjtSensor(model.sensor_type[i]).name
            print(f"  {i+1}. {sensor_name} (类型: {sensor_type})")

    # 初始化雅可比控制相关变量
    jacobian_control = False
    site_id = -1
    if args.jacobian:
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, args.site)
        if site_id < 0:
            print(f"警告: 找不到站点 '{args.site}'，无法启用雅可比控制")
        else:
            jacobian_control = True
            print(f"\n启用雅可比控制，使用站点: {args.site}")
            print(f"末端位姿增量: {args.delta}")
            
            # 打印执行器信息
            if model.nu > 0:
                print(f"\n模型包含 {model.nu} 个执行器:")
                for i in range(model.nu):
                    actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                    print(f"  {i+1}. {actuator_name}")
            else:
                print("\n模型不包含执行器，无法进行雅可比控制")
                jacobian_control = False
    
    # 初始化顺应控制相关变量
    compliance_control = False
    if args.compliance:
        if model.nsensor > 0:
            force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_force")
            torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_torque")
            
            if force_id >= 0 and torque_id >= 0:
                compliance_control = True
                print(f"\n启用顺应控制")
                print(f"力传感器增益: {args.force_gain}")
                print(f"力传感器阈值: {args.force_threshold}")
            else:
                print("\n警告: 找不到必要的力/力矩传感器，无法启用顺应控制")
        else:
            print("\n警告: 模型不包含传感器，无法启用顺应控制")
    
    # 启动查看器
    print("\n启动 MuJoCo 查看器...")
    print("控制提示:")
    print("  - 按 'Esc' 退出")
    print("  - 按 'Space' 暂停/继续模拟")
    print("  - 按 'T' 切换传感器数据显示")
    print("  - 按 '[' 和 ']' 调整传感器图表垂直缩放")
    print("  - 按 ';' 和 \"'\" 调整传感器图表水平缩放")
    print("  - 按 ',' 和 '.' 调整模拟速度")
    print("  - 鼠标左键拖动旋转视角")
    print("  - 鼠标右键拖动平移视角")
    print("  - 鼠标滚轮缩放视角")
    
    # 初始化传感器数据收集器
    data_collector = None
    if args.plot:
        data_collector = SensorDataCollector()
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置查看器参数
        viewer.cam.distance = 1.5  # 相机距离
        viewer.cam.azimuth = 90    # 相机方位角
        viewer.cam.elevation = -20 # 相机仰角
        
        # 添加雅可比控制变量
        delta_pos = np.array([args.delta, 0, 0])  # 默认在x方向上移动
        last_jacobian_time = 0
        jacobian_interval = 0.1  # 每0.1秒应用一次雅可比控制
        
        # 添加顺应控制变量
        last_compliance_time = 0
        compliance_interval = 0.05  # 每0.05秒应用一次顺应控制
        
        # 模拟循环
        start_time = time.time()
        sim_time = 0
        last_display_time = 0
        
        while viewer.is_running():
            # 检查是否达到指定的模拟时间
            if args.duration > 0 and sim_time >= args.duration:
                break
                
            # 步进模拟
            mujoco.mj_step(model, data)
            sim_time = data.time
            
            # 应用顺应控制
            if compliance_control and sim_time - last_compliance_time >= compliance_interval:
                # 获取力传感器数据
                force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_force")
                torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_torque")
                
                force_adr = model.sensor_adr[force_id]
                force_data = data.sensordata[force_adr:force_adr+3]
                
                torque_adr = model.sensor_adr[torque_id]
                torque_data = data.sensordata[torque_adr:torque_adr+3]
                
                # 获取末端站点ID
                site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "dummy_left/ft_sensor_site")
                
                if site_id >= 0:
                    # 获取传感器站点的旋转矩阵（从局部坐标系到世界坐标系）
                    site_xmat = data.site_xmat[site_id].reshape(3, 3)
                    
                    # 将力从传感器坐标系转换到世界坐标系
                    force_world = -site_xmat @ force_data
                    torque_world = -site_xmat @ torque_data
                    
                    # 定义重力方向单位向量 (世界坐标系中指向下方)
                    gravity_direction = np.array([0, 0, -1])
                    
                    # 从力中移除重力方向的分量
                    force_gravity_component = np.dot(force_world, gravity_direction) * gravity_direction
                    force_world_no_gravity = force_world - force_gravity_component
                    
                    # 打印局部坐标系和世界坐标系下的力
                    print(f"局部坐标系力: {force_data}")
                    print(f"世界坐标系力(含重力): {force_world}")
                    print(f"世界坐标系力(去除重力): {force_world_no_gravity}")
                    
                    # 计算去除重力后的力的大小
                    force_magnitude = np.linalg.norm(force_world_no_gravity)
                    
                    # 如果力超过阈值，则响应
                    if force_magnitude > args.force_threshold:
                        # 计算世界坐标系中力的方向（单位向量）
                        force_direction = force_world_no_gravity / force_magnitude
                        
                        # 根据力的方向和大小计算末端位移
                        delta_pos = force_direction * force_magnitude * args.force_gain
                        
                        # 获取雅可比矩阵
                        jacp = np.zeros((3, model.nv))  # 位置雅可比
                        jacr = np.zeros((3, model.nv))  # 旋转雅可比
                        
                        # 计算站点的雅可比矩阵
                        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
                        
                        # 伪逆计算关节速度
                        jac_pinv = np.linalg.pinv(jacp)
                        dq = jac_pinv @ delta_pos
                        
                        # 应用到左侧机械臂的执行器控制
                        left_arm_actuator_indices = range(8)  # 左侧机械臂的8个执行器
                        
                        # 获取当前关节位置
                        current_qpos = data.qpos.copy()
                        
                        # 只更新左侧机械臂的控制信号
                        for i, actuator_idx in enumerate(left_arm_actuator_indices):
                            if i < len(dq):  # 确保不超出dq的范围
                                # 获取执行器对应的关节ID
                                joint_id = model.actuator_trnid[actuator_idx, 0]
                                
                                # 获取关节在qpos中的索引
                                qpos_adr = model.jnt_qposadr[joint_id]
                                
                                # 更新控制信号
                                data.ctrl[actuator_idx] = current_qpos[qpos_adr] + dq[i]
                        
                        # 打印当前力和位置信息
                        site_pos = data.site_xpos[site_id]
                        print(f"时间: {sim_time:.2f}s, 世界坐标系力: {force_world}, 末端位置: {site_pos}")
                
                last_compliance_time = sim_time
            
            # 应用雅可比控制 (如果启用了顺应控制，则不应用雅可比控制)
            if jacobian_control and not compliance_control and sim_time - last_jacobian_time >= jacobian_interval:
                # 获取雅可比矩阵
                jacp = np.zeros((3, model.nv))  # 位置雅可比
                jacr = np.zeros((3, model.nv))  # 旋转雅可比
                
                # 计算站点的雅可比矩阵
                mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
                
                # 只使用位置雅可比（简化版本）
                # 伪逆计算关节速度
                jac_pinv = np.linalg.pinv(jacp)
                dq = jac_pinv @ delta_pos
                
                # 应用到左侧机械臂的执行器控制
                # 根据XML文件，左侧机械臂的执行器索引为0-7
                left_arm_actuator_indices = range(8)  # 左侧机械臂的8个执行器
                
                # 获取当前关节位置
                current_qpos = data.qpos.copy()
                
                # 只更新左侧机械臂的控制信号
                for i, actuator_idx in enumerate(left_arm_actuator_indices):
                    if i < len(dq):  # 确保不超出dq的范围
                        # 获取执行器对应的关节ID
                        joint_id = model.actuator_trnid[actuator_idx, 0]
                        
                        # 获取关节在qpos中的索引
                        qpos_adr = model.jnt_qposadr[joint_id]
                        
                        # 更新控制信号
                        data.ctrl[actuator_idx] = current_qpos[qpos_adr] + dq[i]
                
                last_jacobian_time = sim_time
                
                # 打印当前末端位置
                site_pos = data.site_xpos[site_id]
                print(f"时间: {sim_time:.2f}s, 末端位置: {site_pos}")
            
            # 同步查看器
            viewer.sync()
            
            # 控制模拟速度
            # Rudimentary time keeping, will drift relative to wall clock
            time_until_next_step = model.opt.timestep - (time.time() - start_time) % model.opt.timestep
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            
            # 每帧更新传感器数据
            if args.plot and model.nsensor > 0:
                # 获取力传感器数据
                force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_force")
                torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_torque")
                
                if force_id >= 0 and torque_id >= 0:
                    force_adr = model.sensor_adr[force_id]
                    force_data = data.sensordata[force_adr:force_adr+3]
                    
                    torque_adr = model.sensor_adr[torque_id]
                    torque_data = data.sensordata[torque_adr:torque_adr+3]
                    
                    # 更新数据收集器
                    data_collector.update(force_data, torque_data)
                    
                    # 每0.1秒更新一次文本显示
                    if sim_time - last_display_time >= 0.1:
                        print(data_collector.get_formatted_display())
                        last_display_time = sim_time
            
            # 每秒打印一次传感器数据（可选）
            if not args.plot and int(sim_time) > int(sim_time - model.opt.timestep):
                if model.nsensor > 0:
                    print(f"\n时间: {sim_time:.2f}s")
                    # Get force sensor data
                    force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_force")
                    if force_id >= 0:
                        force_adr = model.sensor_adr[force_id]
                        force_data = data.sensordata[force_adr:force_adr+3]  # Force has 3 components
                        print(f"  左夹爪力传感器: {force_data}")
                    
                    # Get torque sensor data  
                    torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "left_gripper_torque")
                    if torque_id >= 0:
                        torque_adr = model.sensor_adr[torque_id]
                        torque_data = data.sensordata[torque_adr:torque_adr+3]  # Torque has 3 components
                        print(f"  左夹爪力矩传感器: {torque_data}")
        elapsed = time.time() - start_time
        print(f"\n模拟结束. 模拟时间: {sim_time:.2f}s, 实际时间: {elapsed:.2f}s")

if __name__ == "__main__":
    main()
