import os
import sys
import time
import datetime
import json
import struct
import csv
import paho.mqtt.client as mqtt

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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

def curve_traj_cmd(client_, turn_type, turn_dir, v, a):
    cmd_array = [99,109,100,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    cmd_array[3] = 103 # id
    
    cmd_array[5] = turn_type
    if turn_type == 1:
        cmd_array[6] = 1
    elif turn_type == -1:
        cmd_array[6] = 128

    v_int = int(v * 1000)
    cmd_array[7] = v_int & 0x00FF
    cmd_array[8] = (v_int >> 8) & 0x00FF

    a_int = int(a * 1000)
    cmd_array[9] = a_int & 0x00FF
    cmd_array[10] = (a_int >> 8) & 0x00FF

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

        V_L = parse_float(msg.payload[29], msg.payload[28], 5000.0)
        V_R = parse_float(msg.payload[31], msg.payload[30], 5000.0)
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

        if (not (-0.01 < V_L < 0.01)) and (not (-0.01 < V_R < 0.01) ) and (traj_type == 5):
            output_list.append("%f, %f, %f, %f, %f, %f, %f, %f\n" % (V_L, V_R, (V_L + V_R)/2, (V_L - V_R)/2, v, a, target_ang_v, target_ang_a))
            ff_v_coef = 0.3511
            ff_v_offset = 0.8178
            V_uniform_velo = ff_v_coef * target_v + ff_v_offset
            V_acc_velo = (V_L + V_R)/2 - V_uniform_velo
            measure_list.append([V_L, V_R,V_acc_velo , (V_R - V_L)/2, v, a, target_ang_v, target_ang_a])




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


def guruguru(v_max, client_):
    # 走行パラメータ
    v_0 = 0.0
    v_end = 0.1
    a = 5
    x = 1 * 0.09
    stop_time = 0.3
    run_num = 50
    turn_dir = 1 
    turn_type = 1 # TURN_90

    straight_traj_cmd(client_, x, v_0, v_max, v_end, a)
    for ele in range(0, run_num):
        curve_traj_cmd(client_, turn_type, turn_dir, v_max, a)

    straight_traj_cmd(client_, x, v_max, v_max, v_end, a)
    stop_traj_cmd(client_, stop_time)



def main():
    # コンフィグファイルの読み込み
    load_config()

    # MQTTクライアントの初期化
    client = create_mqtt_client()
    client.loop_start()
    
    now = datetime.datetime.now()
    save_file_name =  now.strftime('%Y%m%d_%H%M%S') + "_trans_acc_rot_measure.csv"
    
    guruguru(0.2, client)
    guruguru(0.25, client)
    guruguru(0.3, client)
    guruguru(0.35, client)


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
        
        if stop_count > 5:
            break




    clf = linear_model.LinearRegression()
    #print(measure_list)
    Y = np.array([row[3] for row in measure_list])
    ang_v_target = np.array([row[6] for row in measure_list])
    ang_a_target = np.array([row[7] for row in measure_list])
    X_ang_a = ang_a_target
    X_ang_v = ang_v_target
    
    X_ang_a = np.reshape(X_ang_a,(-1, 1))
    X_ang_v = np.reshape(X_ang_v,(-1, 1))
    X_ang_va = np.c_[X_ang_v, X_ang_a]
    Y = np.reshape(Y,(-1, 1))

    # ang_a vs Volatage
    clf.fit(X_ang_a, Y)
    print("■■ ang_a vs Voltage")
    print("回帰係数=", clf.coef_)
    print("切片：",clf.intercept_)
    print("R^2=",clf.score(X_ang_a, Y))
    plt.scatter(X_ang_a, Y)    
    plt.plot(X_ang_a, clf.predict(X_ang_a))
    plt.grid()    
    plt.show()

    # ang_v vs Voltage
    clf.fit(X_ang_v, Y)
    print("■■ ang_v vs Voltage")
    print("回帰係数=", clf.coef_)
    print("切片：",clf.intercept_)
    print("R^2=",clf.score(X_ang_v, Y))
    plt.scatter(X_ang_v, Y)    
    plt.plot(X_ang_v, clf.predict(X_ang_v))
    plt.grid()
    plt.show()

    # 3D plot    
    clf.fit(X_ang_va, Y)
    print("■■ ang_v, ang_a vs vs Voltage")
    print("回帰係数=", clf.coef_)
    print("切片：",clf.intercept_)
    print("R^2=",clf.score(X_ang_va, Y))

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter3D(X_ang_v, X_ang_a, Y, c = 'red')

    #3D描画（回帰平面の描画）
    mesh_X, mesh_Y = np.meshgrid(np.arange(0, 1000, 50), np.arange(-20000, 20000, 50))
    Z = clf.coef_[0][0] * mesh_X + clf.coef_[0][1] * mesh_Y + clf.intercept_
    ax.plot_surface(mesh_X, mesh_Y, Z, alpha = 0.5) #alphaで透明度を指定
    ax.set_xlabel("ang_v[deg]")
    ax.set_ylabel("ang_a[deg]")
    ax.set_zlabel("(V_L - V_R)/2")
    plt.show()


    #sns.jointplot(X, Y, kind='kde').set_axis_labels('x', 'y')
    #plt.show()

    with open(save_file_name, "w") as f:
        f.writelines(output_list)
        


if __name__ == "__main__":
    main()