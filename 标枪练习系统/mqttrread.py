import random
import pandas as pd
import matplotlib.pyplot as plt
from paho.mqtt import client as mqtt_client
import sys  # 导入 sys 模块，用于退出程序

broker = 'broker.emqx.io'
port = 1883
topic = "test/esp32"
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'stm32'
password = 'stm32stm32'

# 用于存储传感器数据的列表
data = []
data_count = 0  # 用来计数接收到的数据次数
max_data_count = 20  # 收集到20条数据后进行保存和可视化

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(topic)  # 连接后订阅 topic
        else:
            print(f"Failed to connect, return code {rc}")

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    try:
        client.connect(broker, port, 60)
    except Exception as e:
        print(f"Connection failed: {e}")
        return None
    return client

def on_message(client, userdata, msg):
    global data, data_count
    try:
        # 假设收到的消息是以 JSON 格式传递的
        payload = msg.payload.decode()  # 解码消息负载
        data_dict = eval(payload)  # 将 JSON 字符串转换为字典，实际使用时可以使用 json.loads()

        # 获取传感器数据
        roll = data_dict.get('roll', 0)
        pitch = data_dict.get('pitch', 0)
        acceleration = data_dict.get('acceleration', 0)

        # 获取当前时间
        current_time = pd.to_datetime('now')

        # 添加“次数”作为第一列
        data.append((data_count + 1, current_time, roll, pitch, acceleration))  # 第一个元素为计数
        data_count += 1  # 增加数据计数

        # 如果收集到 20 次数据，保存并可视化
        if data_count >= max_data_count:
            # 将数据保存为 Excel
            save_to_excel(data)

            # 将数据转换为 DataFrame 进行可视化
            df = pd.DataFrame(data, columns=['Count', 'Time', 'Roll', 'Pitch', 'Acceleration'])

            # 打印最近的 5 条数据（调试用）
            print(df.tail())

            # 可视化
            plot_data(df)

            # 停止 MQTT 客户端循环
            client.loop_stop()  # 停止循环

            # 退出程序
            print("Received 20 data points, stopping the program.")
            sys.exit(0)  # 退出程序

    except Exception as e:
        print(f"Error processing message: {e}")

# 可视化函数
def plot_data(df):
    plt.figure(figsize=(10, 6))

    # 绘制角度变化
    plt.subplot(2, 1, 1)
    plt.plot(df['Time'], df['Roll'], label='Roll', color='r')
    plt.plot(df['Time'], df['Pitch'], label='Pitch', color='b')
    plt.xlabel('Time')
    plt.ylabel('Angle (Degrees)')
    plt.title('Roll and Pitch Angle over Time')
    plt.legend()

    # 绘制加速度变化
    plt.subplot(2, 1, 2)
    plt.plot(df['Time'], df['Acceleration'], label='Acceleration', color='g')
    plt.xlabel('Time')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Acceleration over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

# 保存数据到 Excel
def save_to_excel(data):
    df = pd.DataFrame(data, columns=['Count', 'Time', 'Roll', 'Pitch', 'Acceleration'])
    df.to_excel('sensor_data.xlsx', index=False)
    print("Data saved to sensor_data.xlsx")

def run():
    client = connect_mqtt()
    if client:
        client.on_message = on_message  # 设置接收到消息时的处理函数
        client.loop_forever()  # 使用阻塞式循环，确保客户端持续运行

if __name__ == '__main__':
    run()
