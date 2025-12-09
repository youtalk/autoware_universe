# Autoware Launch Architecture

This document describes the launch file hierarchy and ComposableNodeContainer relationships in Autoware Universe.

## 1. Launch File Hierarchy

### 1.1 Top-Level Overview

```mermaid
graph TB
    subgraph autoware_launch
        A[autoware.launch.xml]
    end

    subgraph components
        B[tier4_vehicle_component]
        C[tier4_system_component]
        D[tier4_map_component]
        E[tier4_sensing_component]
        F[tier4_localization_component]
        G[tier4_perception_component]
        H[tier4_planning_component]
        I[tier4_control_component]
        J[tier4_autoware_api_component]
    end

    subgraph tier4_launch_packages
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

    A --> B
    A --> C
    A --> D
    A --> E
    A --> F
    A --> G
    A --> H
    A --> I
    A --> J

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

### 1.2 Perception Launch Hierarchy

```mermaid
graph TB
    subgraph tier4_perception_launch
        P[perception.launch.xml]

        subgraph obstacle_segmentation
            GS[ground_segmentation.launch.py]
        end

        subgraph occupancy_grid_map
            OGM[probabilistic_occupancy_grid_map.launch.xml]
        end

        subgraph object_recognition
            subgraph detection
                DET[detection.launch.xml]

                subgraph detectors
                    LIDAR_DNN[lidar_dnn_detector.launch.xml]
                    CAM_LIDAR[camera_lidar_detector.launch.xml]
                    CAM_BEV[camera_bev_detector.launch.xml]
                    LIDAR_RULE[lidar_rule_detector.launch.xml]
                    CAM_VRU[camera_vru_detector.launch.xml]
                end

                subgraph filters
                    PC_MAP[pointcloud_map_filter.launch.py]
                    OBJ_VAL[object_validator.launch.xml]
                    OBJ_FIL[object_filter.launch.xml]
                    RADAR_FIL[radar_filter.launch.xml]
                end

                subgraph mergers
                    CAM_LIDAR_M[camera_lidar_merger.launch.xml]
                    LIDAR_M[lidar_merger.launch.xml]
                end
            end

            TRK[tracking.launch.xml]
            PRED[prediction.launch.xml]
        end

        subgraph traffic_light_recognition
            TL[traffic_light.launch.xml]
            TL_CONTAINER[traffic_light_node_container.launch.py]
            TL_MAP[traffic_light_map_based_detector.launch.py]
            TL_CAM[traffic_light_camera_info_relay.launch.py]
            TL_OCC[traffic_light_occlusion_predictor.launch.py]
        end
    end

    P --> GS
    P --> OGM
    P --> DET
    P --> TRK
    P --> PRED
    P --> TL

    DET --> LIDAR_DNN
    DET --> CAM_LIDAR
    DET --> CAM_BEV
    DET --> LIDAR_RULE
    DET --> CAM_VRU
    DET --> PC_MAP
    DET --> OBJ_VAL
    DET --> OBJ_FIL
    DET --> RADAR_FIL
    DET --> CAM_LIDAR_M
    DET --> LIDAR_M

    TL --> TL_CONTAINER
    TL --> TL_MAP
    TL --> TL_CAM
    TL --> TL_OCC
```

### 1.3 Localization Launch Hierarchy

```mermaid
graph TB
    subgraph tier4_localization_launch
        L[localization.launch.xml]

        subgraph pose_twist_estimator
            PTE[pose_twist_estimator.launch.xml]
            NDT[ndt_scan_matcher.launch.xml]
            YABLOC[yabloc.launch.xml]
            GYRO[gyro_odometer.launch.xml]
            ARTAG[ar_tag_based_localizer.launch.xml]
            LIDAR_MARKER[lidar_marker_localizer.launch.xml]

            subgraph eagleye
                EAG[eagleye_rt.launch.xml]
                GNSS[gnss_converter.launch.xml]
            end
        end

        subgraph pose_twist_fusion_filter
            PTFF[pose_twist_fusion_filter.launch.xml]
        end

        subgraph localization_error_monitor
            LEM[localization_error_monitor.launch.xml]
        end

        subgraph util
            UTIL[util.launch.xml]
        end
    end

    L --> PTE
    L --> PTFF
    L --> LEM

    PTE -->|use_ndt_pose| NDT
    PTE -->|use_yabloc_pose| YABLOC
    PTE -->|use_gyro_odom_twist| GYRO
    PTE -->|use_artag_pose| ARTAG
    PTE -->|use_lidar_marker_pose| LIDAR_MARKER
    PTE -->|use_eagleye| EAG
    PTE --> UTIL

    EAG --> GNSS
```

### 1.4 Sensing Launch Hierarchy

```mermaid
graph TB
    subgraph tier4_sensing_launch
        S[sensing.launch.xml]
    end

    subgraph sensor_kit_launch["Sensor Kit (dynamic)"]
        SK[sensor_kit_launch/sensing.launch.xml]

        subgraph lidar_preprocessing
            PC_CONTAINER[pointcloud_container]
            CONCAT[concatenate_pointclouds]
            CROP[crop_box_filter]
            DISTORT[distortion_corrector]
        end
    end

    S -->|"$(var sensor_model)_launch"| SK
    SK --> PC_CONTAINER
    PC_CONTAINER --> CONCAT
    PC_CONTAINER --> CROP
    PC_CONTAINER --> DISTORT
```

---

## 2. ComposableNodeContainer Relationships

### 2.1 Container Overview

```mermaid
graph TB
    subgraph Containers["ComposableNodeContainers"]
        PC[pointcloud_container<br/><i>sensing</i>]
        TL[traffic_light_node_container<br/><i>perception</i>]
        GS[ground_segmentation_container<br/><i>perception</i>]
    end

    subgraph External["External Loaders"]
        LOC_UTIL[localization/util]
        LOC_MARKER[lidar_marker_localizer]
        PERC_GS[ground_segmentation]
        PERC_IRR[irregular_object_detector]
    end

    LOC_UTIL -->|LoadComposableNodes| PC
    LOC_MARKER -->|LoadComposableNodes| PC
    PERC_GS -->|LoadComposableNodes| PC
    PERC_IRR -->|LoadComposableNodes| PC
```

### 2.2 pointcloud_container Components

```mermaid
graph TB
    subgraph pointcloud_container["pointcloud_container (sensing)"]
        direction TB

        subgraph core["Core Components (sensor_kit)"]
            SYNC[PointCloudDataSynchronizerComponent]
            CONCAT[PointCloudConcatenationComponent]
            CROP_CORE[CropBoxFilterComponent]
            DISTORT[DistortionCorrectorComponent]
            RING[RingOutlierFilterComponent]
        end

        subgraph localization_util["Loaded by: localization/util.launch.xml"]
            CROP_MEAS[CropBoxFilterComponent<br/>crop_box_filter_measurement_range]
            VOXEL[VoxelGridDownsampleFilterComponent<br/>voxel_grid_downsample_filter]
            RANDOM[RandomDownsampleFilterComponent<br/>random_downsample_filter]
        end

        subgraph lidar_marker["Loaded by: lidar_marker_localizer.launch.xml"]
            CROP_MARKER[CropBoxFilterComponent<br/>crop_box_filter_measurement_range]
            PASS[PassThroughFilterUInt16Component<br/>ring_filter]
        end

        subgraph ground_seg["Loaded by: ground_segmentation.launch.py"]
            SCAN_GND[ScanGroundFilterComponent]
            CROP_GND[CropBoxFilterComponent]
            CONCAT_GND[PointCloudConcatenateDataSynchronizerComponent]
        end
    end

    SYNC --> CONCAT
    CONCAT --> CROP_CORE
    CROP_CORE --> DISTORT
    DISTORT --> RING

    CROP_MEAS --> VOXEL
    VOXEL --> RANDOM

    CROP_MARKER --> PASS
```

### 2.3 traffic_light_node_container Components

```mermaid
graph TB
    subgraph traffic_light_node_container["traffic_light_node_container (perception)"]
        direction TB

        subgraph core_nodes["Core Nodes (always loaded)"]
            CAR_CLS[TrafficLightClassifierNodelet<br/>car_traffic_light_classifier]
            PED_CLS[TrafficLightClassifierNodelet<br/>pedestrian_traffic_light_classifier]
            VIS[TrafficLightRoiVisualizerNode<br/>traffic_light_roi_visualizer]
        end

        subgraph conditional["Conditional Loaders"]
            subgraph decompressor["enable_image_decompressor"]
                DECOMP[ImageTransportDecompressor<br/>traffic_light_image_decompressor]
            end

            subgraph fine_detector["enable_fine_detection"]
                FINE[TrafficLightFineDetectorNode<br/>traffic_light_fine_detector]
            end

            subgraph whole_img["whole_img_detector_loader"]
                YOLOX[TrtYoloXNode<br/>traffic_light_whole_image_detector]
                SELECT[TrafficLightSelectorNode<br/>traffic_light_selector]
                MERGE[TrafficLightCategoryMergerNode<br/>traffic_light_category_merger]
            end
        end
    end

    CAR_CLS --> VIS
    PED_CLS --> VIS
    DECOMP -.->|optional| CAR_CLS
    DECOMP -.->|optional| PED_CLS
    YOLOX --> SELECT
    SELECT --> MERGE
    FINE -.->|optional| MERGE
```

---

## 3. Data Flow Through Containers

### 3.1 Pointcloud Processing Pipeline

```mermaid
flowchart LR
    subgraph Input
        LIDAR1[/LiDAR 1/]
        LIDAR2[/LiDAR 2/]
        LIDAR3[/LiDAR N/]
    end

    subgraph pointcloud_container
        SYNC[Time Sync]
        CONCAT[Concatenate]
        CROP[Crop Box]
        DISTORT[Distortion<br/>Corrector]
        RING[Ring Outlier<br/>Filter]
    end

    subgraph ground_segmentation
        GND[Scan Ground<br/>Filter]
    end

    subgraph localization_preprocessing
        VOXEL[Voxel Grid<br/>Downsample]
        RANDOM[Random<br/>Downsample]
    end

    subgraph Output
        NDT[/NDT Scan Matcher/]
        DETECT[/Object Detection/]
        OGM[/Occupancy Grid/]
    end

    LIDAR1 --> SYNC
    LIDAR2 --> SYNC
    LIDAR3 --> SYNC
    SYNC --> CONCAT
    CONCAT --> CROP
    CROP --> DISTORT
    DISTORT --> RING

    RING --> GND
    RING --> VOXEL

    GND --> DETECT
    GND --> OGM

    VOXEL --> RANDOM
    RANDOM --> NDT
```

### 3.2 Object Detection Pipeline (Radar)

```mermaid
flowchart LR
    subgraph Input
        RADAR[/Radar Objects/]
    end

    subgraph radar_filter["radar_filter.launch.xml (Current: 3 processes)"]
        VEL_SPLIT[ObjectVelocitySplitter<br/><i>separate process</i>]
        RANGE_SPLIT[ObjectRangeSplitter<br/><i>separate process</i>]
        LANELET[LaneletFilter<br/><i>separate process</i>]
    end

    subgraph Output
        OUT[/Filtered Objects/]
    end

    RADAR --> VEL_SPLIT
    VEL_SPLIT -->|high_speed| RANGE_SPLIT
    VEL_SPLIT -->|low_speed| OUT
    RANGE_SPLIT -->|far| LANELET
    RANGE_SPLIT -->|near| OUT
    LANELET --> OUT
```

---

## 4. Container Consolidation Opportunities

### 4.1 Current State vs Proposed State

```mermaid
graph TB
    subgraph current["Current State"]
        direction TB
        C_PC[pointcloud_container]
        C_TL[traffic_light_container]
        C_P1[Process: VelocitySplitter]
        C_P2[Process: RangeSplitter]
        C_P3[Process: LaneletFilter]
        C_P4[Process: ObjectValidator]
        C_P5[Process: PositionFilter]
    end

    subgraph proposed["Proposed State"]
        direction TB
        P_PC[pointcloud_container]
        P_TL[traffic_light_container]
        P_OBJ[object_detection_container]

        subgraph P_OBJ_inner["object_detection_container"]
            P_SPLIT[ObjectSplitter<br/><i>unified</i>]
            P_LANELET[LaneletFilter]
            P_VAL[ObjectValidator]
            P_POS[PositionFilter]
        end
    end

    current -->|"Consolidation"| proposed
```

### 4.2 Process Count Comparison

| Module | Current Processes | Proposed Processes | Reduction |
|--------|------------------|-------------------|-----------|
| Radar Filter Pipeline | 3 | 1 | 66% |
| Object Validation | 2-3 | 1 | 50-66% |
| Ground Segmentation | 1 (container) | 1 (container) | - |
| Traffic Light | 1 (container) | 1 (container) | - |

---

## 5. Key File Locations

### Launch Files

| Package | Path |
|---------|------|
| autoware_launch | `/home/yutakakondo/src/autoware/src/launcher/autoware_launch/autoware_launch/launch/` |
| tier4_perception_launch | `launch/tier4_perception_launch/launch/` |
| tier4_localization_launch | `launch/tier4_localization_launch/launch/` |
| tier4_sensing_launch | `launch/tier4_sensing_launch/launch/` |

### Key Container Definitions

| Container | Defined In |
|-----------|------------|
| pointcloud_container | Sensor kit packages (e.g., sample_sensor_kit_launch) |
| traffic_light_node_container | `tier4_perception_launch/launch/traffic_light_recognition/traffic_light_node_container.launch.py` |
| ground_segmentation components | `tier4_perception_launch/launch/obstacle_segmentation/ground_segmentation/ground_segmentation.launch.py` |

### Reference Patterns (Dual-Mode Launch)

- `perception/autoware_ground_segmentation/launch/scan_ground_filter.launch.py` - Standalone OR load into external container pattern

---

## 6. Launch Arguments Flow

```mermaid
flowchart TB
    subgraph autoware_launch
        A_ARGS[Launch Arguments<br/>map_path, vehicle_model,<br/>sensor_model, use_sim_time]
    end

    subgraph component_launch
        C_ARGS[Component Args<br/>perception_mode,<br/>data_path, etc.]
    end

    subgraph tier4_launch
        T_ARGS[Tier4 Launch Args<br/>pointcloud_container_name,<br/>use_intra_process]
    end

    subgraph individual_nodes
        N_ARGS[Node Parameters<br/>YAML config files]
    end

    A_ARGS --> C_ARGS
    C_ARGS --> T_ARGS
    T_ARGS --> N_ARGS
```
