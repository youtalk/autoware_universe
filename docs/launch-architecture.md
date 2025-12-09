# Autoware Launch Architecture

本ドキュメントはAutoware Universeにおけるlaunchファイルの階層構造とComposableNodeContainerの関係を説明する。

---

## 1. Launch File Hierarchy

### 1.1 Top-Level Overview

Autowareの起動は`autoware.launch.xml`をエントリーポイントとして、各コンポーネントのlaunchファイルを呼び出す3層構造になっている。

- **Layer 1**: `autoware_launch` - ユーザー向けのトップレベルlaunchファイル
- **Layer 2**: `tier4_*_component` - autoware_launch内のコンポーネント別ラッパー
- **Layer 3**: `tier4_*_launch` - autoware_universe内の実際のlaunchパッケージ

```mermaid
graph TB
    subgraph autoware_launch["autoware_launch (Layer 1)"]
        A[autoware.launch.xml]
    end

    subgraph components["Component Wrappers (Layer 2)"]
        B[vehicle_component]
        C[system_component]
        D[map_component]
        E[sensing_component]
        F[localization_component]
        G[perception_component]
        H[planning_component]
        I[control_component]
        J[api_component]
    end

    subgraph tier4_launch["tier4_*_launch (Layer 3)"]
        K[tier4_vehicle_launch]
        L[tier4_system_launch]
        M[tier4_map_launch]
        N[tier4_sensing_launch]
        O[tier4_localization_launch]
        P[tier4_perception_launch]
        Q[tier4_planning_launch]
        R[tier4_control_launch]
        S[tier4_autoware_api_launch]
    end

    A --> B & C & D & E & F & G & H & I & J

    B --> K
    C --> L
    D --> M
    E --> N
    F --> O
    G --> P
    H --> Q
    I --> R
    J --> S
```

各コンポーネントは`launch_*`フラグ（例: `launch_perception=true`）で起動を制御できる。

---

### 1.2 Perception Launch Hierarchy

Perceptionモジュールは最も複雑な階層構造を持ち、以下の4つの主要サブシステムで構成される：

1. **obstacle_segmentation** - 地面分離と障害物点群抽出
2. **occupancy_grid_map** - 占有格子地図生成
3. **object_recognition** - 物体検出・追跡・予測
4. **traffic_light_recognition** - 信号機認識

#### 1.2.1 Perception Main Structure

```mermaid
graph TB
    P[perception.launch.xml]

    GS[ground_segmentation]
    OGM[occupancy_grid_map]
    OR[object_recognition]
    TL[traffic_light_recognition]

    P --> GS
    P --> OGM
    P --> OR
    P --> TL
```

#### 1.2.2 Object Recognition - Detection

物体検出は複数のDetector、Filter、Mergerで構成される。`perception_mode`パラメータにより、使用するDetectorの組み合わせが決まる。

```mermaid
graph TB
    DET[detection.launch.xml]

    subgraph detectors["Detectors"]
        D1[lidar_dnn_detector]
        D2[camera_lidar_detector]
        D3[lidar_rule_detector]
        D4[camera_bev_detector]
    end

    subgraph filters["Filters"]
        F1[pointcloud_map_filter]
        F2[object_validator]
        F3[object_filter]
        F4[radar_filter]
    end

    subgraph mergers["Mergers"]
        M1[camera_lidar_merger]
        M2[lidar_merger]
    end

    DET --> detectors
    DET --> filters
    DET --> mergers
```

#### 1.2.3 Traffic Light Recognition

信号機認識は専用のComposableNodeContainer（`traffic_light_node_container`）を使用する。

```mermaid
graph TB
    TL[traffic_light.launch.xml]

    C[traffic_light_node_container]
    MAP[map_based_detector]
    CAM[camera_info_relay]
    OCC[occlusion_predictor]

    TL --> C
    TL --> MAP
    TL --> CAM
    TL --> OCC
```

---

### 1.3 Localization Launch Hierarchy

Localizationは複数の自己位置推定手法をサポートしており、`pose_source`と`twist_source`パラメータで選択する。

- **pose_source**: `ndt`, `yabloc`, `eagleye`, `artag`, `lidar-marker`（複数選択可、アンダースコア区切り）
- **twist_source**: `gyro_odom`, `eagleye`

#### 1.3.1 Localization Main Structure

```mermaid
graph TB
    L[localization.launch.xml]

    PTE[pose_twist_estimator]
    PTFF[pose_twist_fusion_filter]
    LEM[localization_error_monitor]

    L --> PTE
    L --> PTFF
    L --> LEM
```

#### 1.3.2 Pose/Twist Estimator Options

各推定手法は条件付きで起動される。複数手法を選択した場合は`pose_estimator_arbiter`が調停を行う。

```mermaid
graph TB
    PTE[pose_twist_estimator]

    NDT[ndt_scan_matcher]
    YAB[yabloc]
    EAG[eagleye]
    ART[ar_tag_based_localizer]
    LID[lidar_marker_localizer]
    GYR[gyro_odometer]
    UTIL[util.launch.xml]

    PTE -->|"ndt in pose_source"| NDT
    PTE -->|"yabloc in pose_source"| YAB
    PTE -->|"eagleye in pose_source"| EAG
    PTE -->|"artag in pose_source"| ART
    PTE -->|"lidar-marker in pose_source"| LID
    PTE -->|"twist_source=gyro_odom"| GYR
    PTE --> UTIL
```

---

### 1.4 Sensing Launch Hierarchy

Sensingは`sensor_model`パラメータで指定されたセンサーキットパッケージに処理を委譲する。
センサーキットパッケージが`pointcloud_container`（ComposableNodeContainer）を作成し、各種フィルタをロードする。

```mermaid
graph TB
    S[sensing.launch.xml]

    SK["$(sensor_model)_launch"]
    PC[pointcloud_container]

    SYNC[Time Synchronizer]
    CONCAT[Concatenation]
    CROP[CropBox Filter]
    DIST[Distortion Corrector]

    S -->|"動的にパッケージ選択"| SK
    SK --> PC
    PC --> SYNC
    PC --> CONCAT
    PC --> CROP
    PC --> DIST
```

---

## 2. ComposableNodeContainer Relationships

### 2.1 Container Overview

Autowareでは主に以下のComposableNodeContainerが使用される。複数のlaunchファイルが`LoadComposableNodes`を使って同一コンテナにノードをロードすることで、プロセス間通信（IPC）を削減している。

```mermaid
graph TB
    subgraph containers["Main Containers"]
        PC[("pointcloud_container")]
        TL[("traffic_light_node_container")]
    end

    subgraph loaders["LoadComposableNodes呼び出し元"]
        L1[sensor_kit]
        L2[localization/util]
        L3[lidar_marker_localizer]
        L4[ground_segmentation]
        L5[irregular_object_detector]
    end

    L1 --> PC
    L2 --> PC
    L3 --> PC
    L4 --> PC
    L5 --> PC
```

**重要**: `pointcloud_container`は複数のモジュール（sensing, localization, perception）から共有されており、intra-process通信によりゼロコピーでデータを受け渡す。

---

### 2.2 pointcloud_container Components

`pointcloud_container`には複数のlaunchファイルからComposableNodeがロードされる。

#### 2.2.1 Core Components (sensor_kit)

センサーキットパッケージで作成される基本的な点群処理パイプライン。

```mermaid
graph LR
    SYNC[PointCloudData<br/>Synchronizer] --> CONCAT[PointCloud<br/>Concatenation]
    CONCAT --> CROP[CropBox<br/>Filter]
    CROP --> DISTORT[Distortion<br/>Corrector]
    DISTORT --> RING[RingOutlier<br/>Filter]
```

#### 2.2.2 Localization Preprocessing (util.launch.xml)

NDT Scan Matcher用のダウンサンプリング処理。点群数を削減して自己位置推定の計算負荷を軽減する。

```mermaid
graph LR
    CROP[CropBoxFilter<br/>measurement_range] --> VOXEL[VoxelGridDownsample<br/>Filter]
    VOXEL --> RANDOM[RandomDownsample<br/>Filter]
```

#### 2.2.3 LiDAR Marker Localizer (lidar_marker_localizer.launch.xml)

反射マーカーによる自己位置推定用の前処理。

```mermaid
graph LR
    CROP[CropBoxFilter<br/>measurement_range] --> PASS[PassThroughFilter<br/>ring_filter]
```

#### 2.2.4 Ground Segmentation (ground_segmentation.launch.py)

地面点群の分離処理。障害物検出とOccupancy Grid Map生成に使用される。

```mermaid
graph LR
    CROP[CropBoxFilter] --> SCAN[ScanGround<br/>Filter]
    CONCAT[PointCloudConcatenate<br/>DataSynchronizer] --> SCAN
```

#### 2.2.5 Container Loading Summary

```mermaid
graph TB
    PC[("pointcloud_container")]

    subgraph loaders["LoadComposableNodes Sources"]
        L1[sensor_kit]
        L2[localization/util]
        L3[lidar_marker_localizer]
        L4[ground_segmentation]
    end

    L1 -->|"Sync, Concat,<br/>Crop, Distort, Ring"| PC
    L2 -->|"Crop, Voxel, Random"| PC
    L3 -->|"Crop, PassThrough"| PC
    L4 -->|"Crop, ScanGround, Concat"| PC
```

---

### 2.3 traffic_light_node_container Components

信号機認識専用のコンテナ。カメラごとにインスタンスが作成される。

#### 2.3.1 Core Nodes (常にロード)

```mermaid
graph LR
    CAR[car_traffic_light<br/>_classifier] --> VIS[roi_visualizer]
    PED[pedestrian_traffic<br/>_light_classifier] --> VIS
```

#### 2.3.2 Conditional Nodes (オプション)

```mermaid
graph TB
    subgraph optional["条件付きロード"]
        DECOMP[ImageTransport<br/>Decompressor]
        FINE[FineDetector]

        subgraph whole["whole_img_detector_loader"]
            YOLOX[TrtYoloX] --> SELECT[Selector]
            SELECT --> MERGE[CategoryMerger]
        end
    end
```

**条件**:
- `enable_image_decompressor=true` → Decompressorをロード
- `enable_fine_detection=true` → FineDetectorをロード

---

## 3. Data Flow Through Containers

### 3.1 Pointcloud Processing Pipeline

複数のLiDARからの点群が統合され、各サブシステムへ分岐する。

```mermaid
graph TB
    subgraph input["Input"]
        L1[LiDAR 1]
        L2[LiDAR 2]
        L3[LiDAR N]
    end

    subgraph preprocessing["pointcloud_container"]
        SYNC[Time Sync]
        CONCAT[Concatenate]
        CROP[CropBox]
        RING[RingOutlier]
    end

    subgraph output["Output Branches"]
        GND[Ground<br/>Segmentation]
        LOC[Localization<br/>Downsample]
    end

    subgraph consumers["Consumers"]
        NDT[NDT Scan<br/>Matcher]
        DET[Object<br/>Detection]
        OGM[Occupancy<br/>Grid Map]
    end

    L1 & L2 & L3 --> SYNC
    SYNC --> CONCAT --> CROP --> RING

    RING --> GND --> DET & OGM
    RING --> LOC --> NDT
```

---

### 3.2 Object Detection Pipeline (Radar)

現在のradar_filter.launch.xmlでは、3つの独立プロセスが直列に接続されている。これはIssue #11738で指摘されているアンチパターンの例。

```mermaid
graph LR
    RADAR[/Radar<br/>Objects/]

    VEL[Velocity<br/>Splitter]
    RANGE[Range<br/>Splitter]
    LANE[Lanelet<br/>Filter]

    OUT[/Filtered<br/>Objects/]

    RADAR --> VEL
    VEL -->|high_speed| RANGE
    VEL -->|low_speed| OUT
    RANGE -->|far| LANE
    RANGE -->|near| OUT
    LANE --> OUT
```

**問題点**: 各ノードが別プロセスとして動作し、IPC通信のオーバーヘッドが発生する。

---

## 4. Container Consolidation Opportunities

### 4.1 Current State vs Proposed State

Issue #11738で提案されているコンテナ統合の概要。

#### Current State (現状)

```mermaid
graph TB
    subgraph current["現状: 複数の独立プロセス"]
        C_PC[pointcloud_container]
        C_TL[traffic_light_container]
        C_P1[Process: VelocitySplitter]
        C_P2[Process: RangeSplitter]
        C_P3[Process: LaneletFilter]
        C_P4[Process: ObjectValidator]
    end
```

#### Proposed State (提案)

```mermaid
graph TB
    subgraph proposed["提案: コンテナ統合"]
        P_PC[pointcloud_container]
        P_TL[traffic_light_container]
        P_OBJ[object_detection_container]
    end

    subgraph obj_inner["object_detection_container内"]
        P_SPLIT[ObjectSplitter<br/>統合版]
        P_LANE[LaneletFilter]
        P_VAL[ObjectValidator]
    end

    P_OBJ --- obj_inner
```

### 4.2 Process Count Comparison

| モジュール | 現状プロセス数 | 提案プロセス数 | 削減率 |
|-----------|--------------|--------------|-------|
| Radar Filter Pipeline | 3 | 1 | 66% |
| Object Validation | 2-3 | 1 | 50-66% |
| Ground Segmentation | 1 (container) | 1 (container) | - |
| Traffic Light | 1 (container) | 1 (container) | - |

### 4.3 Benefits of Consolidation

1. **メモリ削減**: プロセス毎のアドレス空間オーバーヘッドを削減
2. **レイテンシ削減**: IPC通信をintra-process通信（ゼロコピー）に置換
3. **デバッグ容易化**: プロセス数が減り、ログ追跡が簡単に
4. **リソース効率**: スレッドプール共有による効率化

---

## 5. Key File Locations

### 5.1 Launch Files

| パッケージ | パス |
|-----------|------|
| autoware_launch | `autoware_launch/autoware_launch/launch/` |
| tier4_perception_launch | `launch/tier4_perception_launch/launch/` |
| tier4_localization_launch | `launch/tier4_localization_launch/launch/` |
| tier4_sensing_launch | `launch/tier4_sensing_launch/launch/` |

### 5.2 Key Container Definitions

| コンテナ | 定義ファイル |
|---------|------------|
| pointcloud_container | Sensor kit packages (例: sample_sensor_kit_launch) |
| traffic_light_node_container | `tier4_perception_launch/launch/traffic_light_recognition/traffic_light_node_container.launch.py` |

### 5.3 Reference Patterns

**Dual-Mode Launch Pattern** (スタンドアロン または 外部コンテナへロード):
- `perception/autoware_ground_segmentation/launch/scan_ground_filter.launch.py`

このパターンは`container`引数が空の場合は独自コンテナを作成し、指定がある場合は`LoadComposableNodes`で外部コンテナにロードする。

---

## 6. Launch Arguments Flow

Launch引数はトップレベルから各ノードのパラメータまで階層的に伝播する。

```mermaid
graph TB
    subgraph L1["Layer 1: autoware_launch"]
        A1["map_path<br/>vehicle_model<br/>sensor_model<br/>use_sim_time"]
    end

    subgraph L2["Layer 2: component_launch"]
        A2["perception_mode<br/>data_path<br/>launch_* flags"]
    end

    subgraph L3["Layer 3: tier4_launch"]
        A3["pointcloud_container_name<br/>use_intra_process"]
    end

    subgraph L4["Layer 4: Individual Nodes"]
        A4["YAML config files<br/>node parameters"]
    end

    L1 --> L2 --> L3 --> L4
```

---

## 7. Related Documents

- [Issue #11738: Node/Callback Explosion Anti-pattern](https://github.com/autowarefoundation/autoware.universe/issues/11738)
- [Anti-pattern Analysis Report](../.claude/anti-pattern-analysis-report.md)
- [Callback Reduction Design Plan](../.claude/plans/wise-noodling-cascade.md)
