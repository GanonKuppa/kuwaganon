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

x = 0.0
y = 0.0
ang = 0.0
la = 0.0
r = 0.0
l = 0.0
ra = 0.0


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

        x = parse_float(msg.payload[93], msg.payload[92], 1000.0)
        y = parse_float(msg.payload[95], msg.payload[94], 1000.0)
        ang = parse_float(msg.payload[97], msg.payload[96], 100.0)
        
        la = parse_int(msg.payload[59], msg.payload[58])
        r = parse_int(msg.payload[61], msg.payload[60])
        l = parse_int(msg.payload[63], msg.payload[62])
        ra = parse_int(msg.payload[65], msg.payload[64])
        print("%f, %f, %f, %d, %d, %d, %d" % (x,y,ang,la,r,l,ra))





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


    print("x, y, ang, la, r, l , ra")
    while True:
        time.sleep(0.1)
        
        
                


if __name__ == "__main__":
    main()