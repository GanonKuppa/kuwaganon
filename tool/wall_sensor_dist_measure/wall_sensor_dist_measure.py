import os
import sys
import time
import datetime
import json
import struct
import csv
import paho.mqtt.client as mqtt

import matplotlib.pyplot as plt
import seaborn as sns
from sklearn import linear_model
from sklearn import datasets
import numpy as np
import pandas as pd




# 設定系変数(デフォルト値で初期化)
MQTT_BROKER_IP = "DEVNOTEPC"
MQTT_BROKER_PORT = 1883
VAL_NUM = 240

x = 0.0
y = 0.0
ang = 0.0
la = 0.0
r = 0.0
l = 0.0
ra = 0.0

rec_list = []
output_list = []

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
        for i in range(7,240):
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
        wall_2_mouse_center = 0.152 - y
        print("%f, %f, %d, %d, %d, %d" % (wall_2_mouse_center,ang,la,r,l,ra))
        rec_list.append([wall_2_mouse_center, la, ra])
        output_list.append("%f, %d, %d\n" % (wall_2_mouse_center,la,ra) )

def straight_traj_cmd(client_, x, v_0, v_max, v_end, a):
    cmd_array = [99,109,100,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    cmd_array[3] = 101 # id

    x_int = int(x * 1000)
    cmd_array[5] = x_int & 0x00FF
    cmd_array[6] = (x_int >> 8) & 0x00FF

    v_0_int = int(v_0 * 1000)
    cmd_array[7] = v_0_int & 0x00FF
    cmd_array[8] = (v_0_int >> 8) & 0x00FF

    v_max_int = int(v_max * 1000)
    cmd_array[9] = v_max_int & 0x00FF
    cmd_array[10] = (v_max_int >> 8) & 0x00FF

    v_end_int = int(v_end * 1000)
    cmd_array[11] = v_end_int & 0x00FF
    cmd_array[12] = (v_end_int >> 8) & 0x00FF

    a_int = int(a * 1000)
    cmd_array[13] = a_int & 0x00FF
    cmd_array[14] = (a_int >> 8) & 0x00FF

    checksum = sum(cmd_array[5:]) % 256

    cmd_array[4] = checksum
    print(x_int, v_0_int, v_max_int, v_end_int, a_int)
    client_.publish("cmd", bytearray(cmd_array))

def stop_traj_cmd(client_, stop_time):
    cmd_array = [99,109,100,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    cmd_array[3] = 100 # id
    stop_time_int = int(stop_time * 1000)
    cmd_array[5] = stop_time_int & 0x00FF
    cmd_array[6] = (stop_time_int >> 8) & 0x00FF
    checksum = sum(cmd_array[5:]) % 256
    cmd_array[4] = checksum
    client_.publish("cmd", bytearray(cmd_array))

def pos_ang_cmd(client_, x, y, ang):
    cmd_array = [99,109,100,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    cmd_array[3] = 255 # id  
    x_int = int(x * 1000)
    cmd_array[5] = x_int & 0x00FF
    cmd_array[6] = (x_int >> 8) & 0x00FF
    y_int = int(y * 1000)
    cmd_array[7] = y_int & 0x00FF
    cmd_array[8] = (y_int >> 8) & 0x00FF
    ang_int = int(ang * 1)    
    cmd_array[9] = ang_int & 0x00FF
    cmd_array[10] = (ang_int >> 8) & 0x00FF
    
    
    checksum = sum(cmd_array[5:]) % 256
    cmd_array[4] = checksum
    client_.publish("cmd", bytearray(cmd_array))


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


    v_0 = 0.0
    v_end = 0.1
    v_max = 0.1
    a = 5.0
    x = 0.09 + 0.01765
    stop_time = 0.3

    pos_ang_cmd(client, 0, 0, 90.0)
    straight_traj_cmd(client, x, v_0, v_max, v_end, a)
    stop_traj_cmd(client, stop_time)
    #print("x, y, ang, la, r, l , ra")
    
    elapsed_time = 0.0
    while elapsed_time < 1.5:        
        time.sleep(0.1)
        elapsed_time = elapsed_time + 0.1
    print("============================")
    print(rec_list)
    DIST = np.array([row[0] for row in rec_list])
    LA = np.array([row[1] for row in rec_list])
    RA = np.array([row[2] for row in rec_list])
    LA = np.reshape(LA,(-1, 1))
    RA = np.reshape(RA,(-1, 1))

    plt.scatter(DIST, LA)
    plt.scatter(DIST, RA)

    now = datetime.datetime.now()
    save_file_name =  now.strftime('%Y%m%d_%H%M%S') + "_ahead_wall_dist.csv"
    with open(save_file_name, "w") as f:
        f.writelines(output_list)


    # 回帰直線    
    plt.grid()
    plt.show()



if __name__ == "__main__":
    main()