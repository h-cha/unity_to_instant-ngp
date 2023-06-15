# unity_to_instant-ngp
## data
### 230613
- 230613_remocon/png   
  - スマホで撮影した元画像（画像サイズ　3024 × 4032）   
- 230613_remocon/png1   
  -  charuco用に画像をリサイズ（画像サイズ　1080 × 1920）   
- 230613_remocon/transforms.json   
  - `unity_generate_transform_center_change_camera.py`を使用してunityで取得した座標を変換して出力   
  - 230613_remocon/png1のデータを使用   
- 230613_remocon/transforms_estimate_charuco.json   
  - charucoボードを使用してカメラ姿勢を推定   
  - 230613_remocon/png1のデータを使用   
- 230613_remocon/xarm_position_xyz.txt
  -  https://github.com/h-cha/unity_to_instant-ngp/issues/1#issuecomment-1591113184
  -  ロボットアームの先端の座標を取得
  -  `1.00000  0.00031  -0.00050  0.00031　←xarm.transform.rotation　Quaternion(x, y, z, w)`
  -  `-0.28  0.35  0.28　←xarm.transform.position　(x, y, z)`

### 230615
- 230615_unity/xarm_position_xyz.txt
  -  
  -  仮のカメラを４つ配置して、それぞれの座標を取得
