# Akari_simulator
Akari simulator for Rviz2

## Akari_ros2_humbleをRviz2で動かすプログラム
Akari_ros2_humbleは[こちら](https://github.com/AkariGroup/akari_ros)

---
### rvizdisplay.launch.py
Rviz2でAkariを動かすことができます。

use_guiを入力しないと動きません

#### GUIでRviz2のAkariを動かす
```
 ros2 launch rvizdisplay.launch.py use_gui:=True
```
#### Akariの動作とRviz2のモデルを同期したいとき
```
 ros2 launch rvizdisplay.launch.py use_gui:=False
```
---
### rviz_sim.launch.py
AkariがなくてもRviz2で動かすことができるプログラム（作成中そのうちUnityと連携します。）

サーバーのlaunch
```
rviz_sim.launch.py
```

このあと下記の動作があります。

サーボを動かすプログラム
```
ros2 run akari_simulator sim_servo_client.py
```
Akariが挨拶をするプログラム
```
ros2 run akari_simulator sim_servo_client.py
```
