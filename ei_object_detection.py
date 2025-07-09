import sensor, image, time, os, tf, math, uos, gc
from pyb import UART
import struct

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.skip_frames(time=2000)

# 初始化串口通信
uart = UART(2, 115200)

# 加载模型
net = None
labels = None
min_confidence = 0.4

try:
    net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite": ' + str(e))

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt": ' + str(e))

# 颜色定义和物体类型映射
COLORS = [
    (255, 0, 0),    # 红色 - 未使用
    (0, 255, 0),    # 绿色 - 斑马线
    (255, 255, 0),  # 黄色 - 盲道
    (0, 0, 255),    # 蓝色 - 红灯
    (255, 0, 255),  # 紫色 - 绿灯
]

# 物体类型ID映射
OBJECT_TYPE_ID = {
    "ZebraCrossing": 3,
    "BlindPath": 4,
    "RedLight": 1,
    "GreenLight": 2,
    "Unknown": 0
}

# 区域边界定义
LEFT_BOUNDARY = 80
RIGHT_BOUNDARY = 160

# 橙黄色阈值(HSV)
ORANGE_YELLOW_THRESHOLD = [(20, 100, 15, 127, 15, 127)]

# 物体优先级(值越小优先级越高)
OBJECT_PRIORITY = {
    "RedLight": 1,
    "GreenLight": 2,
    "ZebraCrossing": 3,
    "BlindPath": 4,
    "Unknown": 5
}

# 字符映射：物体类型 -> 输出字符
CHAR_MAPPING = {
    "RedLight": "A",
    "GreenLight": "B",
    "ZebraCrossing": "CC",
    "BlindPath": "DD",
    "Unknown": "X"  # 未知物体的默认输出
}

# 数据打包函数：将位置和物体类型转换为指定格式的字符串
def pack_data(object_type, position):
    # 获取物体对应的字符
    obj_char = CHAR_MAPPING.get(object_type, "X")
    return obj_char, str(position)

# 离线模式检测
def is_offline_mode():
    try:
        os.listdir("/sd")
        return False
    except:
        return True

clock = time.clock()
last_send_time = 0
RETRY_INTERVAL = 5000  # 修改为5秒(5000ms)
last_detection_time = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    current_time = time.ticks_ms()
    offline_mode = is_offline_mode()

    # 检查是否达到5秒间隔
    if time.ticks_diff(current_time, last_send_time) < RETRY_INTERVAL:
        continue  # 如果不到5秒，跳过本次循环

    # 初始化最高优先级物体
    highest_priority = 5
    main_object = None  # (Name, Position, Type, CenterX, CenterY, Color)

    # 物体检测
    for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
        if i == 0: continue  # 跳过背景
        if len(detection_list) == 0: continue

        for d in detection_list:
            x, y, w, h = d.rect()
            center_x = math.floor(x + w/2)
            center_y = math.floor(y + h/2)

            # 确定位置
            if center_x < LEFT_BOUNDARY:
                position = 1
            elif center_x > RIGHT_BOUNDARY:
                position = 2
            else:
                position = 0

            # 获取物体类型
            obj_name = "Unknown"
            if i < len(COLORS):
                color = COLORS[i]
                if i == 1: obj_name = "ZebraCrossing"
                elif i == 2: obj_name = "BlindPath"
                elif i == 3: obj_name = "RedLight"
                elif i == 4: obj_name = "GreenLight"

            obj_type = OBJECT_TYPE_ID.get(obj_name, 0)
            priority = OBJECT_PRIORITY.get(obj_name, 5)

            # 更新最高优先级物体
            if priority < highest_priority:
                highest_priority = priority
                main_object = (obj_name, position, obj_type, center_x, center_y, color)

    # 颜色识别(盲道检测)
    for blob in img.find_blobs(ORANGE_YELLOW_THRESHOLD, pixels_threshold=200, area_threshold=200):
        center_x = blob.cx()
        center_y = blob.cy()

        # 确定位置
        if center_x < LEFT_BOUNDARY:
            position = 1
        elif center_x > RIGHT_BOUNDARY:
            position = 2
        else:
            position = 0

        obj_name = "BlindPath"
        obj_type = OBJECT_TYPE_ID["BlindPath"]
        color = COLORS[2]
        priority = OBJECT_PRIORITY["BlindPath"]

        # 更新最高优先级物体
        if priority < highest_priority:
            highest_priority = priority
            main_object = (obj_name, position, obj_type, center_x, center_y, color)

    # 绘图和数据传输
    if main_object:
        obj_name, position, obj_type, center_x, center_y, color = main_object
        img.draw_circle(center_x, center_y, 5, color=color)

        # 分别打包物体类型和位置
        obj_char, pos_str = pack_data(obj_name, position)

        # 在终端打印
        print(obj_char)  # 打印物体类型
        print(pos_str)   # 打印位置

        # 通过串口发送数据
        uart.write(obj_char + "\n")  # 发送物体类型
        uart.write(pos_str + "\n")   # 发送位置

        last_send_time = current_time
        last_detection_time = current_time
    else:
        # 未检测到物体时发送默认状态
        obj_char, pos_str = pack_data("Unknown", 0)
        print(obj_char)  # 打印物体类型
        print(pos_str)   # 打印位置
        uart.write(obj_char + "\n")  # 发送物体类型
        uart.write(pos_str + "\n")   # 发送位置
        last_send_time = current_time
