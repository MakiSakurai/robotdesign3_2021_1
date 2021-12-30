# RobotDesign3_2021_1

## パッケージについて
このパッケージは[オリジナル](https://github.com/rt-net/crane_x7_ros)である株式会社アールティ様のパッケージを使用して、千葉工業大学未来ロボティクス学科の2021年度の設計製作論3という講義で知能コース1班が作成したものです。このリポジトリは株式会社アールティ様のライセンスに則って作成しています。詳細は、LICENSEファイルをご参照ください。

## 概要

このリポジトリは、株式会社アールティ様が販売されているcrane_x7を制御しナイフ刺しをさせるパッケージです。
このパッケージを使用する際カメラが必要になりますが、今回はRealSenseD435の使用を前提としています。

## 動作環境
OS : Ubuntu 18.04LTS

ROS Distribution: Melodic Morenia

Gazebo 9.0.0

Rviz 1.13.21

## セットアップ方法

- gitを使用して、robotdesign3_2021_1をダウロードします
```
cd ~/catkin_ws/src
git clone https://github.com/MakiSakurai/robotdesign3_2021_1.git
```

- 株式会社アールティ様から配布されているcrane_x7_rosをダウロードします
```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/crane_x7_ros.git
```

- このパッケージは、RealSenseD435を使用することを想定しているので、RealSenseD435のシミュレーターのモデルの適用のため、Kuwamai様が公開されているcrane_x7_d435をダウロードします
```
cd ~/catkin_ws/src
git clone https://github.com/Kuwamai/crane_x7_d435.git
```

- catkin buildを使用して本パッケージをビルドします
```
cd ~/catkin
catkin build
source ~/catkin_ws/devel/setup.bash
```

- 今回カメラを使用しますが、[robotdesign3_2021_1/hand_coordinates.pyの18行目](https://github.com/MakiSakurai/robotdesign3_2021_1/blob/main/hand_coordinates.py#:~:text=cap%20%3D%20cv2.VideoCapture(-,4,-))を使用しているカメラが認識されている/dev/videoの番号に変更してください


## 使用方法

### シミュレーター起動用コマンド

```
roslaunch robotdesign3_2021_1 main_sim.launch
```

### 実機起動用コマンド

crane_x7をPCに接続し、以下のコマンドを実行してデバイスドライバに実行権限を与えてから起動します
```
sudo chmod 666 /dev/ttyUSB0
roslaunch robotdesign3_2021_1 main.launch
```

### キーボード操作一覧

```
o: グリッパーを開く
c: グリッパーを閉める
f: verticalへ移動
h: ホームポジションへ移動
q: 親指・人差し指の間へ移動し刺す
w: 人差し指・中指の間へ移動し刺す
e: 中指・薬指の間へ移動し刺す
r: 人差し指・中指の間へ移動し刺す
l: 親指・人差し指の間への移動をループ
```

## 参考用

以下の手順で動かすと、安定して指すことができます

![スクリーンショット 2021-12-27 232127](https://user-images.githubusercontent.com/71488377/147767505-4808c209-2b6e-456e-90a0-8ad1a7e12a70.png)


### 動いている様子

https://youtu.be/zFIe7depjOk
