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



V_L = 0.0
V_R = 0.0
v = 0.0
a = 0.0
target_ang_v = 0.0
omega = 0.0
alpha = 0.0
output_list = []
measure_list = []

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

    

def stop_traj_cmd(client_, stop_time):
    cmd_array = [99,109,100,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    cmd_array[3] = 100 # id
    stop_time_int = int(stop_time * 1000)
    cmd_array[5] = stop_time_int & 0x00FF
    cmd_array[6] = (stop_time_int >> 8) & 0x00FF
    checksum = sum(cmd_array[5:]) % 256
    cmd_array[4] = checksum
    client_.publish("cmd", bytearray(cmd_array))


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

def spinturn_traj_cmd(client_, ang, ang_v, ang_a):
    cmd_array = [99,109,100,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    cmd_array[3] = 102 # id

    ang_int = int(ang * 1)
    cmd_array[5] = ang_int & 0x00FF
    cmd_array[6] = (ang_int >> 8) & 0x00FF

    ang_v_int = int(ang_v * 1)
    cmd_array[7] = ang_v_int & 0x00FF
    cmd_array[8] = (ang_v_int >> 8) & 0x00FF

    ang_a_int = int(ang_a * 1)
    cmd_array[9] = ang_a_int & 0x00FF
    cmd_array[10] = (ang_a_int >> 8) & 0x00FF

    checksum = sum(cmd_array[5:]) % 256

    cmd_array[4] = checksum
    client_.publish("cmd", bytearray(cmd_array))

def steady_state_circular_traj_cmd(client_, ang, ang_v, v):
    cmd_array = [99,109,100,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    cmd_array[3] = 104 # id

    ang_int = int(ang * 1)
    cmd_array[5] = ang_int & 0x00FF
    cmd_array[6] = (ang_int >> 8) & 0x00FF

    ang_v_int = int(ang_v * 1)
    cmd_array[7] = ang_v_int & 0x00FF
    cmd_array[8] = (ang_v_int >> 8) & 0x00FF

    v_int = int(v * 1000)
    cmd_array[9] = v_int & 0x00FF
    cmd_array[10] = (v_int >> 8) & 0x00FF

    checksum = sum(cmd_array[5:]) % 256

    cmd_array[4] = checksum
    client_.publish("cmd", bytearray(cmd_array))


def on_message(client, userdata, msg):    
    if msg.topic == "mouse":
        check_sum = 0
        for i in range(7,240):
            check_sum += msg.payload[i]
        check_sum = check_sum % 256
        if check_sum != msg.payload[6]:
            #print("check_sum error!")
            return

        duty_L = parse_float(msg.payload[25], msg.payload[24], 10000.0)
        duty_R = parse_float(msg.payload[27], msg.payload[26], 10000.0)
        v = parse_float(msg.payload[107], msg.payload[106], 10000.0)
        a = parse_float(msg.payload[159], msg.payload[158], 1000.0)
        target_v = parse_float(msg.payload[111], msg.payload[110], 10000.0)
        target_a = parse_float(msg.payload[123], msg.payload[122], 1000.0)
        target_ang_v = parse_float(msg.payload[117], msg.payload[116], 10.0)
        target_ang_a = parse_float(msg.payload[87], msg.payload[86], 1.0)
        traj_type = parse_int(msg.payload[81], msg.payload[80])
        omega = parse_float(msg.payload[115], msg.payload[114], 10.0) * 3.14159265 / 180.0
        alpha = parse_float(msg.payload[157], msg.payload[156], 2.0) * 3.14159265 / 180.0
        #print("%f, %f, %f, %f, %f, %f" % (V_L, V_R, v, a, target_v, target_a))

        if (not (-0.01 < duty_L < 0.01)) and (not (-0.01 < duty_R < 0.01) ) and not (-0.01 < target_a < 0.01) and (target_v > 0.05):
            output_list.append("%f, %f, %f, %f, %f, %f, %f, %f\n" % (duty_L, duty_R, (duty_L + duty_R)/2, (duty_L - duty_R)/2, v, a, target_v, target_a))
            ff_v_coef = 0.08
            ff_v_offset = 0.0
            duty_uniform_velo = ff_v_coef * target_v + ff_v_offset
            duty_acc_velo = (duty_L + duty_R)/2 - duty_uniform_velo
            measure_list.append([duty_L, duty_R,duty_acc_velo , (duty_L - duty_R)/2, v, a, target_v, target_a])




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
    
    # 走行パラメータ
    v_0 = 0.0
    v_end = 0.1
    v_max = 2.0
    a = 0.5
    x = 7 * 0.09
    stop_time = 0.3
    run_num = 50
    now = datetime.datetime.now()
    save_file_name =  now.strftime('%Y%m%d_%H%M%S') + "_trans_acc_velo_measure.csv"
    
    for ele in range(0, run_num):
        straight_traj_cmd(client, x, v_0, v_max, v_end, a)
        stop_traj_cmd(client, stop_time)
        spinturn_traj_cmd(client, 180.0, 1200.0, 1200.0)
        stop_traj_cmd(client, stop_time)
        a = a + 0.25


    time_count = 0
    output_list_len = 0
    stop_count = 0
    while True:
        time.sleep(1.0)
        print(len(output_list))
        if output_list_len == len(output_list):
            stop_count = stop_count + 1
        else:
            stop_count = 0
        
        output_list_len = len(output_list)
        
        if stop_count > 20:
            break

    clf = linear_model.LinearRegression()
    print(measure_list)
    Y = np.array([row[2] for row in measure_list])
    v_target = np.array([row[6] for row in measure_list])
    a_target = np.array([row[7] for row in measure_list])
    X = a_target
    print(len(X))
    print(len(Y))
    X = np.reshape(X,(-1, 1))
    Y = np.reshape(Y,(-1, 1))
    clf.fit(X, Y)
    print("回帰係数=", clf.coef_)
    print("切片：",clf.intercept_)
    print("R^2=",clf.score(X, Y))
    plt.scatter(X, Y)
    # 回帰直線    
    
    plt.plot(X, clf.predict(X))
    plt.grid()
    
    plt.show()
    sns.jointplot(X, Y, kind='kde').set_axis_labels('x', 'y')
    plt.show()

    with open(save_file_name, "w") as f:
        f.writelines(output_list)
        


if __name__ == "__main__":
    main()