import os
import sys
import time
import datetime
import json
import struct
import csv
import paho.mqtt.client as mqtt

# 設定系変数(デフォルト値で初期化)
MQTT_BROKER_IP = "DEVNOTEPC"
MQTT_BROKER_PORT = 1883
VAL_NUM = 400


V_L = 0.0
V_R = 0.0
v = 0.0
a = 0.0
omega = 0.0
alpha = 0.0

def load_config():
    with open('config.json', 'r') as f:
        config = json.load(f)
        MQTT_BROKER_IP = config["MQTT_BROKER_IP"]
        MQTT_BROKER_PORT = config["MQTT_BROKER_PORT"]


def parse_int(val1, val0):
    int_val = val1 + val0 * 256
    if int_val > 32767:
        int_val = int_val - 65536
    return int_val


def parse_float(val1, val0, division_scaler):
    int_val = parse_int(val1, val0)
    return int_val / division_scaler



def on_message(client, userdata, msg):    
    if msg.topic == "mouse":
        check_sum = 0
        for i in range(7,400):
            check_sum += msg.payload[i]
        check_sum = check_sum % 256
        if check_sum != msg.payload[6]:
            #print("check_sum error!")
            return

        V_L = parse_float(msg.payload[29], msg.payload[28], 5000.0)
        V_R = parse_float(msg.payload[31], msg.payload[30], 5000.0)
        v = parse_float(msg.payload[107], msg.payload[106], 10000.0)
        a = parse_float(msg.payload[159], msg.payload[158], 1000.0)
        omega = parse_float(msg.payload[115], msg.payload[114], 10.0) * 3.14159265 / 180.0
        alpha = parse_float(msg.payload[157], msg.payload[156], 2.0) * 3.14159265 / 180.0

        if not (-0.01 < V_L < 0.01) or not (-0.01 < V_R < 0.01):
            print("%f, %f, %f, %f, %f, %f" % (V_L, V_R, v, a, omega, alpha))





def create_mqtt_client():
    host = MQTT_BROKER_IP
    port = MQTT_BROKER_PORT

    # インスタンス作成時に protocl v3.1.1を指定
    client = mqtt.Client(protocol=mqtt.MQTTv311)

    client.connect(host, port=port, keepalive=60)
    client.publish("presence","this is " + __file__)
    client.on_message = on_message 
    client.subscribe("mouse")
    return client



def main():
    # コンフィグファイルの読み込み
    load_config()

    # MQTTクライアントの初期化
    client = create_mqtt_client()
    client.loop_start()


    print("V_L, V_R, v, a, omega, alpha")
    while True:
        time.sleep(0.1)
        
        
                


if __name__ == "__main__":
    main()