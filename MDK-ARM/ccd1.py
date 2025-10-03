import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import struct
import numpy as np
from collections import deque
import csv
import os
from datetime import datetime
from PIL import Image, ImageTk


class LinearCCDAdjuster:
    def __init__(self, root):
        self.root = root
        self.root.title("线性CCD调参工具 - 高效版")
        self.root.geometry("1200x800")

        # 串口相关变量
        self.serial_port = None
        self.is_connected = False
        self.receiving = False

        # 数据存储
        self.ccd_data = np.zeros(2000, dtype=np.uint16)
        self.actual_length = 0
        self.data_buffer = bytearray()
        self.receive_count = 0

        # 绘图相关
        self.canvas = None
        self.graph_width = 900
        self.graph_height = 400
        self.margin = 50
        self.max_data = 4095
        # 流动图像相关
        self.waterfall_canvas = None
        self.waterfall_image = None
        self.waterfall_photo = None
        self.waterfall_history = 100  # 默认显示100帧历史
        self.waterfall_data = np.zeros((self.waterfall_history, 2000), dtype=np.uint8)
        self.current_row = 0

        # 性能优化
        self.last_update_time = 0
        self.update_interval = 0.05  # 50ms更新一次显示

        self.setup_ui()

    def setup_ui(self):
        """设置用户界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 串口设置区域
        serial_frame = ttk.LabelFrame(main_frame, text="串口设置", padding="5")
        serial_frame.pack(fill=tk.X, pady=5)

        serial_subframe = ttk.Frame(serial_frame)
        serial_subframe.pack(fill=tk.X)

        ttk.Label(serial_subframe, text="端口:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(serial_subframe, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Label(serial_subframe, text="波特率:").grid(row=0, column=2, padx=5)
        self.baud_combo = ttk.Combobox(
            serial_subframe,
            width=10,
            values=[
                "9600",
                "19200",
                "38400",
                "57600",
                "115200",
                "230400",
                "460800",
                "921600",
            ],
        )
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=3, padx=5)

        ttk.Button(serial_subframe, text="刷新端口", command=self.refresh_ports).grid(
            row=0, column=4, padx=5
        )
        self.connect_btn = ttk.Button(
            serial_subframe, text="打开串口", command=self.toggle_serial
        )
        self.connect_btn.grid(row=0, column=5, padx=5)

        # 数据显示区域
        data_frame = ttk.LabelFrame(main_frame, text="数据信息", padding="5")
        data_frame.pack(fill=tk.X, pady=5)

        data_subframe = ttk.Frame(data_frame)
        data_subframe.pack(fill=tk.X)

        ttk.Label(data_subframe, text="接收帧数:").grid(row=0, column=0, padx=5)
        self.frame_count_label = ttk.Label(data_subframe, text="0", width=8)
        self.frame_count_label.grid(row=0, column=1, padx=5)

        ttk.Label(data_subframe, text="像素数量:").grid(row=0, column=2, padx=5)
        self.pixel_count_label = ttk.Label(data_subframe, text="0", width=8)
        self.pixel_count_label.grid(row=0, column=3, padx=5)

        ttk.Label(data_subframe, text="最大值:").grid(row=0, column=4, padx=5)
        self.max_value_label = ttk.Label(data_subframe, text="0", width=8)
        self.max_value_label.grid(row=0, column=5, padx=5)

        ttk.Label(data_subframe, text="最小值:").grid(row=0, column=6, padx=5)
        self.min_value_label = ttk.Label(data_subframe, text="0", width=8)
        self.min_value_label.grid(row=0, column=7, padx=5)

        ttk.Label(data_subframe, text="帧率(FPS):").grid(row=0, column=8, padx=5)
        self.fps_label = ttk.Label(data_subframe, text="0", width=8)
        self.fps_label.grid(row=0, column=9, padx=5)

        # 控制按钮区域
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=5)

        self.start_btn = ttk.Button(
            control_frame, text="开始接收", command=self.start_receive, state=tk.DISABLED
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(
            control_frame, text="停止接收", command=self.stop_receive, state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(control_frame, text="清除数据", command=self.clear_data).pack(
            side=tk.LEFT, padx=5
        )

        # 新增保存按钮
        ttk.Button(control_frame, text="保存当前帧", command=self.save_current_frame).pack(
            side=tk.LEFT, padx=5
        )

        # 流动图像设置区域
        waterfall_control_frame = ttk.Frame(main_frame)
        waterfall_control_frame.pack(fill=tk.X, pady=5)

        ttk.Label(waterfall_control_frame, text="历史帧数:").pack(side=tk.LEFT, padx=5)
        self.history_scale = ttk.Scale(
            waterfall_control_frame, from_=50, to=500, orient=tk.HORIZONTAL, length=150
        )
        self.history_scale.set(self.waterfall_history)
        self.history_scale.pack(side=tk.LEFT, padx=5)

        ttk.Button(
            waterfall_control_frame, text="更新历史帧数", command=self.update_history_size
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            waterfall_control_frame, text="保存瀑布图", command=self.save_waterfall_image
        ).pack(side=tk.LEFT, padx=5)

        # 创建左右分栏
        paned_window = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True, pady=5)

        # 左侧 - 波形图
        left_frame = ttk.Frame(paned_window)
        paned_window.add(left_frame, weight=1)

        # 右侧 - 瀑布图
        right_frame = ttk.Frame(paned_window)
        paned_window.add(right_frame, weight=1)

        # 波形图区域
        self.setup_plot_area(left_frame)

        # 瀑布图区域
        self.setup_waterfall_area(right_frame)

        # 初始刷新端口
        self.refresh_ports()

        # 帧率计算
        self.frame_times = deque(maxlen=30)

    def setup_plot_area(self, parent):
        """设置波形图区域"""
        plot_frame = ttk.LabelFrame(parent, text="CCD数据波形", padding="5")
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # 创建Canvas
        self.canvas = tk.Canvas(
            plot_frame,
            bg="white",
            width=self.graph_width + 2 * self.margin,
            height=self.graph_height + 2 * self.margin,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 绘制坐标轴
        self.draw_axes()

    def setup_waterfall_area(self, parent):
        """设置瀑布图区域"""
        waterfall_frame = ttk.LabelFrame(parent, text="CCD数据瀑布图", padding="5")
        waterfall_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # 创建Canvas用于显示瀑布图
        self.waterfall_canvas = tk.Canvas(
            waterfall_frame, bg="black", width=900, height=500
        )
        self.waterfall_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 初始化瀑布图图像
        self.init_waterfall_image()

    def init_waterfall_image(self):
        """初始化瀑布图图像"""
        # 创建初始图像（全黑）
        self.waterfall_image = Image.new("L", (2000, self.waterfall_history), 0)
        self.update_waterfall_display()

    def update_history_size(self):
        """更新历史帧数设置"""
        new_history = int(self.history_scale.get())
        if new_history != self.waterfall_history:
            self.waterfall_history = new_history
            # 重新初始化瀑布图数据
            self.waterfall_data = np.zeros(
                (self.waterfall_history, 2000), dtype=np.uint8
            )
            self.current_row = 0
            self.init_waterfall_image()
            print(f"历史帧数已更新为: {self.waterfall_history}")

    def update_waterfall_display(self):
        """更新瀑布图显示"""
        if self.waterfall_image:
            # 调整图像大小以适应Canvas
            canvas_width = self.waterfall_canvas.winfo_width()
            canvas_height = self.waterfall_canvas.winfo_height()

            if canvas_width > 1 and canvas_height > 1:  # 确保Canvas已初始化
                resized_image = self.waterfall_image.resize(
                    (canvas_width, canvas_height), Image.NEAREST
                )
                self.waterfall_photo = ImageTk.PhotoImage(resized_image)
                self.waterfall_canvas.delete("waterfall")
                self.waterfall_canvas.create_image(
                    0, 0, anchor=tk.NW, image=self.waterfall_photo, tags="waterfall"
                )

    def add_frame_to_waterfall(self, data):
        """将当前帧数据添加到瀑布图"""
        if self.actual_length == 0:
            return

        # 将数据归一化到0-255 (假设最大值为4095)
        normalized_data = (data[: self.actual_length] / self.max_data * 255).astype(
            np.uint8
        )

        # 确保数据长度一致
        if len(normalized_data) < 2000:
            padded_data = np.zeros(2000, dtype=np.uint8)
            padded_data[: len(normalized_data)] = normalized_data
            normalized_data = padded_data

        # 将数据向下移动一行
        self.waterfall_data = np.roll(self.waterfall_data, 1, axis=0)

        # 将新数据放在最上面
        self.waterfall_data[0] = normalized_data

        # 更新图像
        self.waterfall_image = Image.fromarray(self.waterfall_data, mode="L")
        self.update_waterfall_display()

    def save_waterfall_image(self):
        """保存瀑布图为图片"""
        if self.waterfall_image is None:
            messagebox.showwarning("警告", "没有可保存的瀑布图数据!")
            return

        # 生成默认文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"ccd_waterfall_{timestamp}.png"

        # 选择保存路径
        file_path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[
                ("PNG files", "*.png"),
                ("JPEG files", "*.jpg"),
                ("All files", "*.*"),
            ],
            initialfile=default_filename,
        )

        if not file_path:
            return  # 用户取消了保存

        try:
            # 保存原始大小的图像，不进行缩放
            self.waterfall_image.save(file_path)
            messagebox.showinfo("成功", f"瀑布图已保存到:\n{file_path}")
            print(f"瀑布图已保存到: {file_path}")
        except Exception as e:
            messagebox.showerror("错误", f"保存瀑布图失败: {str(e)}")
            print(f"保存瀑布图失败: {e}")

    def draw_axes(self):
        """绘制坐标轴和网格"""
        self.canvas.delete("grid")

        # 坐标轴
        self.canvas.create_line(
            self.margin,
            self.margin,
            self.margin,
            self.graph_height + self.margin,
            width=2,
            tags="grid",
        )
        self.canvas.create_line(
            self.margin,
            self.graph_height + self.margin,
            self.graph_width + self.margin,
            self.graph_height + self.margin,
            width=2,
            tags="grid",
        )

        # 网格线
        for i in range(0, 11):
            y = self.margin + i * self.graph_height / 10
            self.canvas.create_line(
                self.margin,
                y,
                self.graph_width + self.margin,
                y,
                dash=(2, 2),
                fill="lightgray",
                tags="grid",
            )

        for i in range(0, 11):
            x = self.margin + i * self.graph_width / 10
            self.canvas.create_line(
                x,
                self.margin,
                x,
                self.graph_height + self.margin,
                dash=(2, 2),
                fill="lightgray",
                tags="grid",
            )

        # 坐标轴标签
        self.canvas.create_text(
            self.margin - 10, self.margin, text="4095", anchor=tk.E, tags="grid"
        )
        self.canvas.create_text(
            self.margin - 10,
            self.graph_height + self.margin,
            text="0",
            anchor=tk.E,
            tags="grid",
        )
        self.canvas.create_text(
            self.margin,
            self.graph_height + self.margin + 15,
            text="0",
            anchor=tk.N,
            tags="grid",
        )
        self.canvas.create_text(
            self.graph_width + self.margin,
            self.graph_height + self.margin + 15,
            text="像素位置",
            anchor=tk.N,
            tags="grid",
        )
        self.canvas.create_text(
            self.margin - 20,
            self.margin + self.graph_height / 2,
            text="灰度值",
            angle=90,
            anchor=tk.CENTER,
            tags="grid",
        )

    def refresh_ports(self):
        """刷新可用串口列表"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo["values"] = port_list
        if port_list:
            if "COM6" in port_list:
                self.port_combo.set("COM6")
            else:
                self.port_combo.set(port_list[0])

    def toggle_serial(self):
        """打开/关闭串口"""
        if not self.is_connected:
            self.open_serial()
        else:
            self.close_serial()

    def open_serial(self):
        """打开串口"""
        try:
            port = self.port_combo.get()
            baudrate = int(self.baud_combo.get())

            if not port:
                messagebox.showerror("错误", "请选择串口!")
                return

            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )

            self.is_connected = True
            self.connect_btn.config(text="关闭串口")
            self.start_btn.config(state=tk.NORMAL)

            # 启动串口数据读取线程
            self.serial_thread = threading.Thread(
                target=self.read_serial_data, daemon=True
            )
            self.serial_thread.start()

            print(f"串口 {port} 已打开")

        except Exception as e:
            messagebox.showerror("错误", f"打开串口失败: {str(e)}")

    def close_serial(self):
        """关闭串口"""
        self.stop_receive()

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        self.is_connected = False
        self.connect_btn.config(text="打开串口")
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        print("串口已关闭")

    def start_receive(self):
        """开始接收数据"""
        if self.is_connected and not self.receiving:
            self.receiving = True
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.data_buffer = bytearray()
            print("开始接收数据...")

    def stop_receive(self):
        """停止接收数据"""
        self.receiving = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        print("停止接收数据")

    def read_serial_data(self):
        """读取串口数据的线程函数"""
        while self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if self.receiving:
                        self.process_received_data(data)
                time.sleep(0.001)
            except Exception as e:
                print(f"读取串口数据错误: {e}")
                if self.is_connected:
                    self.root.after(
                        0, lambda: messagebox.showerror("错误", f"串口通信错误: {e}")
                    )
                break

    def process_received_data(self, data):
        """处理接收到的数据"""
        self.data_buffer.extend(data)

        frame_found = True
        while frame_found and len(self.data_buffer) >= 4000:
            frame_found = False

            # 查找帧头
            for i in range(len(self.data_buffer) - 7):
                if (
                    self.data_buffer[i] == 0x3D
                    and self.data_buffer[i + 1] == 0x7E
                    and self.data_buffer[i + 6] == 0x7E
                    and self.data_buffer[i + 7] == 0x3D
                ):
                    data_mode = 0

                    # 解析数据长度
                    data_length = self.data_buffer[i + 2] + (
                        self.data_buffer[i + 3] << 8
                    )
                    if self.data_buffer[i + 4] == 1:
                        data_mode = 1
                    if data_mode == 1:
                        total_frame_length = 8 + data_length
                        self.max_data = 255
                    else:
                        total_frame_length = 8 + data_length * 2
                        self.max_data = 4095

                    if len(self.data_buffer) - i >= total_frame_length:
                        # 提取数据部分
                        data_start = i + 8
                        if data_mode == 1:
                            data_end = 8 + data_length
                        else:
                            data_end = 8 + data_length * 2
                        data_section = self.data_buffer[data_start:data_end]

                        # 解析CCD数据
                        self.actual_length = min(data_length, len(self.ccd_data))
                        if data_mode == 1:
                            for j in range(self.actual_length):
                                if j < len(data_section):
                                    self.ccd_data[j] = data_section[j]
                        else:
                            for j in range(self.actual_length):
                                if j * 2 + 1 < len(data_section):
                                    self.ccd_data[j] = data_section[j * 2] + (
                                        data_section[j * 2 + 1] << 8
                                    )
                        # 移除已处理的数据
                        self.data_buffer = self.data_buffer[i + total_frame_length :]

                        # 更新显示
                        self.receive_count += 1
                        current_time = time.time()
                        self.frame_times.append(current_time)

                        # 限制更新频率
                        if current_time - self.last_update_time >= self.update_interval:
                            self.last_update_time = current_time
                            self.root.after(0, self.update_display)
                            # self.update_display()

                        frame_found = True
                        break

            # 如果没有找到完整帧，清理缓冲区
            if not frame_found and len(self.data_buffer) > 7:
                self.data_buffer = self.data_buffer[-7:]

    def update_display(self):
        """更新数据显示和绘图"""
        try:
            # 更新数据信息
            self.frame_count_label.config(text=str(self.receive_count))
            self.pixel_count_label.config(text=str(self.actual_length))

            if self.actual_length > 0:
                current_data = self.ccd_data[: self.actual_length]
                max_val = np.max(current_data)
                min_val = np.min(current_data)

                self.max_value_label.config(text=str(max_val))
                self.min_value_label.config(text=str(min_val))

                # 计算帧率
                if len(self.frame_times) > 1:
                    fps = len(self.frame_times) / (
                        self.frame_times[-1] - self.frame_times[0]
                    )
                    self.fps_label.config(text=f"{fps:.1f}")

                # 更新波形图
                self.draw_waveform(current_data, max_val)

                # 更新瀑布图
                self.add_frame_to_waterfall(current_data)

        except Exception as e:
            print(f"更新显示错误: {e}")

    def draw_waveform(self, data, max_val):
        """绘制波形图（比柱状图更高效）"""
        # 清除之前的波形
        self.canvas.delete("waveform")

        if len(data) == 0:
            return

        # 计算缩放比例
        x_scale = self.graph_width / len(data)
        # y_scale = self.graph_height / (max_val if max_val > 0 else 4095)
        y_scale = self.graph_height / (self.max_data)

        # 创建波形点
        points = []
        for i, value in enumerate(data):
            x = self.margin + i * x_scale
            y = self.margin + self.graph_height - (value * y_scale)
            points.extend([x, y])

        # 绘制折线
        if len(points) > 2:
            self.canvas.create_line(
                points, fill="blue", width=1, tags="waveform", smooth=True
            )

        # 在关键位置绘制一些采样点
        step = max(1, len(data) // 50)  # 最多显示50个点
        for i in range(0, len(data), step):
            x = self.margin + i * x_scale
            y = self.margin + self.graph_height - (data[i] * y_scale)
            self.canvas.create_oval(
                x - 2, y - 2, x + 2, y + 2, fill="red", outline="", tags="waveform"
            )

    def save_current_frame(self):
        """保存当前帧数据为CSV文件"""
        if self.actual_length == 0:
            messagebox.showwarning("警告", "没有可保存的数据!")
            return

        # 生成默认文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"ccd_data_{timestamp}.csv"

        # 选择保存路径
        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=default_filename,
        )

        if not file_path:
            return  # 用户取消了保存

        try:
            with open(file_path, "w", newline="", encoding="utf-8") as csvfile:
                writer = csv.writer(csvfile)
                # 写入表头
                writer.writerow(["像素位置", "灰度值"])

                # 写入数据
                for i in range(self.actual_length):
                    writer.writerow([i, self.ccd_data[i]])

            messagebox.showinfo("成功", f"数据已保存到:\n{file_path}")
            print(f"当前帧数据已保存到: {file_path}")

        except Exception as e:
            messagebox.showerror("错误", f"保存文件失败: {str(e)}")
            print(f"保存文件失败: {e}")

    def clear_data(self):
        """清除数据"""
        self.receive_count = 0
        self.ccd_data.fill(0)
        self.actual_length = 0
        self.data_buffer = bytearray()
        self.frame_count_label.config(text="0")
        self.pixel_count_label.config(text="0")
        self.max_value_label.config(text="0")
        self.min_value_label.config(text="0")
        self.fps_label.config(text="0")

        # 清除波形
        self.canvas.delete("waveform")
        self.frame_times.clear()

        # 清除瀑布图
        self.waterfall_data.fill(0)
        self.init_waterfall_image()

    def __del__(self):
        """析构函数，确保串口关闭"""
        if (
            hasattr(self, "serial_port")
            and self.serial_port
            and self.serial_port.is_open
        ):
            self.serial_port.close()


def main():
    root = tk.Tk()
    app = LinearCCDAdjuster(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close_serial(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
