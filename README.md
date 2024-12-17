![Static Badge](https://img.shields.io/badge/ROS-noetic-blue)
# Human_Collaboration_Manipulation_System

# 概要
・ロボット産業IoTイニシアチブ協議会が作成した，人協働マニピュレーション機能インターフェース仕様書ver1.3をもとに，人協働マニピュレーションシステムをROSで実装を行った． 

・仕様書のアクティビティ図を参考に，エラーなどの発生を含まない正常系での動作の実装を行なった．

・開発したパッケージは仕様書に沿った形で利用可能になるため， 様々なROS対応のマニピュレータに適用可能である．  
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

# ディレクトリ構造
```sh
.
├── MOTOMAN #motoman-g8での検証コード
│   ├── discharge_position_detect　#排出位置検出サブシステム
│   ├── human_collaboration #人協働マニピュレーションモジュール
│   │   └── scripts
│   │       └── HumanCollaborationUserDefineModule.py #ユーザ定義ファイル　ロボット依存の情報など
│   ├── peripheral_environment_detection　#周辺環境認識サブシステム
│   ├── system_management　#上位アプリ
│   ├── work_detection　#ワーク検出サブシステム
│   └── ws_recogneze #WS内環境認識サブシステム
└── Skelton #実機の情報がないスケルトンコード
    ├── discharge_position_detect　#排出位置検出サブシステム
    ├── human_collaboration #人協働マニピュレーションモジュール
    │   └── scripts
    │       └── HumanCollaborationUserDefineModule.py #ユーザ定義ファイル　ロボット依存の情報など
    ├── peripheral_environment_detection　#周辺環境認識サブシステム
    ├── system_management　#上位アプリ
    ├── work_detection　#ワーク検出サブシステム
    └── ws_recogneze #WS内環境認識サブシステム

```
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

システムの検証として用いたMOTOMAN-GP8は以下を参照して，環境構築した．

https://wwwms.meijo-u.ac.jp/kohara/technicalreport/ros_motoman_gp8_setup

ハードウェア，実行シナリオを変える場合にはユーザ定義ファイルであるHumanCollaborationUserDefineModule.pyを適宜変更する．

ハードウェア, シミュレータを立ち上げた後に実行するコマンドは以下の通りである. 
```sh
$ rosrun human_collaboration HumanCollaborationModule.py
```

## システムモデル
システム間のデータのやりとりは以下の通りである
![システム間のやりとり](https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/f7af74ba-4859-4642-815d-e2dd9168e4c4)

本システム構成は以下の通りである
![システム構成図](https://github.com/user-attachments/assets/c3d306f9-9727-4a5f-95e3-a2e6d1f7ddba)


# 仕様書との対応部分について
本システムはエラー発生を含まない正常系の動作を実装したものになっている．一方，仕様書はエラーなどの異常系についても定義されているため，仕様書と本パッケージの対応関係について示す．
仕様書と本パッケージの対応関係は，以下のアクティビティ図とステートマシン図のうち、赤枠で囲われた部分になっている．

![人協働_アクティビティ図_適応範囲](https://github.com/user-attachments/assets/af25dda5-7102-46a4-9695-d40cc98d0c3a)
![人協働_状態遷移図_対応応範囲](https://github.com/user-attachments/assets/54859ccc-d1d9-4602-81a3-fd5aa19ef5f4)

# パッケージ概要
各パッケージの機能以下の通りである


## 貢献者
Yukiya Yamamoto ([Yukiya-Yamamoto](https://github.com/Yukiya-Yamamoto))

Itsuku Kito ([Itsuku-Kito](https://github.com/Itsuku-Kito))

