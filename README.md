![Static Badge](https://img.shields.io/badge/ROS-noetic-blue)
# Human_Collaboration_Manipulation_System

# 概要
・ロボット産業IoTイニシアチブ協議会が作成した，人協働マニピュレーション機能インターフェース仕様書ver1.3をもとに,人協働マニピュレーションシステムをROSで実装を行った． 

・システムの実装にあたっては，正常系での動作

・開発したパッケージは仕様書に沿った形で利用可能になるため， 様々なROS対応のマニピュレータに適用可能．  
（一例としてROS対応マニピュレータであるMOTOMAN-GP8による検証を行った）    

# 仕様
**人協働マニピュレーションモジュール**    

| 開発言語 | Python |    
|:------:|:------:|  
| OS | Linux(Ubuntu20.04) | 
| ミドルウェア | ROS noetic |  

# システムのシナリオ
シナリオとして，工場のラインを想定してデモ動画では，行われている．

以下にハードウェア構成とその外観について載せる．
![ハードウェア構成](https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/a0212d4d-917a-4a6c-8967-68e0160c7d13)


動画は以下から参照

![デモ動画](https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/11cac15f-276d-4409-b84b-b1d11332c902)


# インストール方法

このリポジトリを自身の環境に合わせてクローンする
```sh
$ cd catkin_ws/src
$ git clone https://github.com/Yukiya-Yamamoto/test_HumanCollaborationSystem
```

## 各ファイルのビルド
クローンが完了したら，ビルドをおこなう
```sh
$ cd catkin_ws
$ catkin build
```

## 本システムの動作確認
ハードウェアのインストールや設定は，各自環境構築を行う．

実行コマンドは以下の通り
```sh
$ rosrun human_collaboration HumanCollaborationModule.py
```

## システムモデル
システム間のデータのやりとりは以下の通りである
![システム間のやりとり](https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/f7af74ba-4859-4642-815d-e2dd9168e4c4)

本システム構成は以下の通りである

# 仕様書との対応部分について
人協働マニピュレーションインターフェース仕様書と本システムの対応については，アクティビティ図とステートマシン図をもとに，以下の通りとなっている
![人協働_アクティビティ図](https://github.com/user-attachments/assets/5348932b-6f57-4d07-8e7c-6f42b2967b2c)

# パッケージ概要
各パッケージの機能以下の通りである

## 貢献者
Yukiya Yamamoto ([Yukiya-Yamamoto](https://github.com/Yukiya-Yamamoto))

Itsuku Kito ([Itsuku-Kito](https://github.com/Itsuku-Kito))
