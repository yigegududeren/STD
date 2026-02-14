# LIO-SAM Integration Guide for STD

**语言 / Language:** [English](#english) | [中文](#中文)

---

## English

### Overview

This guide explains how to integrate the STD (Stable Triangle Descriptor) loop closure detection module into LIO-SAM for improved SLAM performance through robust loop closure detection and pose graph optimization.

### Table of Contents
1. [Point Cloud Data Requirements](#point-cloud-data-requirements)
2. [Keyframe Selection Strategy](#keyframe-selection-strategy)
3. [Factor Graph Integration](#factor-graph-integration)
4. [Implementation Steps](#implementation-steps)
5. [Code Examples](#code-examples)

---

### Point Cloud Data Requirements

#### What Point Cloud Should Be Passed to STD?

When integrating STD into LIO-SAM, you should pass **keyframe point clouds in the global/world coordinate frame** to the STD descriptor generator.

**Input Requirements:**
- **Coordinate Frame**: World/Global frame (not body/sensor frame)
- **Point Cloud Type**: Accumulated point clouds from multiple scans (keyframes)
- **Data Format**: `pcl::PointCloud<pcl::PointXYZI>`
- **Preprocessing**: Motion-distortion corrected (LIO-SAM already provides this)

**LIO-SAM Provides:**
- `/cloud_registered`: Point cloud in the global frame (already motion-compensated)
- `/cloud_registered_body`: Point cloud in the body frame
- **Use `/cloud_registered` or manually transform `/cloud_registered_body` to world frame**

#### Point Cloud Accumulation Strategy

STD works best with **accumulated point clouds** from multiple consecutive frames. Based on the `online_demo.cpp` implementation:

```cpp
// Accumulate point clouds for keyframes
PointCloud::Ptr key_cloud(new PointCloud);

// For each frame before keyframe threshold
*key_cloud += *current_cloud_world;

// When keyframe is reached (e.g., every 10 frames)
if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
    std_manager->GenerateSTDescs(key_cloud, stds_vec);
    key_cloud->clear(); // Reset for next keyframe
}
```

**Recommendation for LIO-SAM:**
- Accumulate 10-20 frames (depending on your `sub_frame_num` parameter)
- This provides richer geometric information for descriptor extraction
- Ensures sufficient corner features for place recognition

---

### Keyframe Selection Strategy

#### LIO-SAM's Native Keyframe Selection

LIO-SAM selects keyframes based on:
1. **Distance threshold**: New keyframe when robot moves > X meters
2. **Rotation threshold**: New keyframe when robot rotates > Y degrees
3. **Time threshold**: Minimum time between keyframes

#### STD's Keyframe Strategy

STD uses a simpler **frame-count-based** strategy:
- Select every N-th frame as a keyframe (e.g., N=10)
- Accumulate N consecutive frames to form one keyframe point cloud

#### Recommended Integration Strategy

**Use STD's Frame-Count-Based Strategy (Independent of LIO-SAM's Keyframes)**

The key insight is that STD should use its **own frame-count-based keyframe selection**, independent of LIO-SAM's distance/rotation-based keyframe selection. This means:

1. **Accumulate every frame** in world coordinates (from LIO-SAM's odometry)
2. **Count frames** independently of LIO-SAM's keyframe logic
3. **Create STD keyframes** every N frames (e.g., N=10)
4. **Pass accumulated point clouds** to STD

```cpp
// In LIO-SAM's main loop (process every frame, not just LIO-SAM keyframes)
void processPointCloud() {
    // Get current world frame cloud from LIO-SAM
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*currentCloud, *world_cloud, currentPose);
    
    // Accumulate for STD (independent of LIO-SAM keyframe selection)
    accumulated_clouds_for_std.push_back(world_cloud);
    std_frame_count++;
    
    // STD keyframe logic (every N frames, regardless of LIO-SAM keyframes)
    if (std_frame_count >= std_keyframe_interval) {
        // Merge accumulated clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto& cloud : accumulated_clouds_for_std) {
            *merged_cloud += *cloud;
        }
        
        // Generate STD descriptors
        std_manager->GenerateSTDescs(merged_cloud, stds_vec);
        std_manager->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair);
        
        if (search_result.first >= 0) {
            addSTDLoopFactor(current_liosam_id, matched_liosam_id, loop_transform);
        }
        
        std_manager->AddSTDescs(stds_vec);
        
        // Reset for next STD keyframe
        accumulated_clouds_for_std.clear();
        std_frame_count = 0;
    }
}
```

**Why This Approach?**
- STD's frame-count strategy is simpler and more predictable
- Decouples STD processing from LIO-SAM's adaptive keyframe selection
- Ensures consistent temporal spacing for STD descriptors
- Works well with STD's design philosophy

---

### Factor Graph Integration

#### Understanding the Factor Graph Structure

**LIO-SAM's Factor Graph:**
```
Nodes: Pose estimates at keyframes
Edges:
  - Odometry factors (between consecutive keyframes)
  - GPS factors (if available)
  - Loop closure factors (from Scan Context or other methods)
```

**Adding STD Loop Closure Factors:**

STD provides:
1. **Loop detection**: Identifies which historical keyframe matches the current one
2. **Relative transformation**: Computes the 6-DOF transformation between matched frames
3. **Geometric verification**: Uses plane-to-plane ICP for refinement

#### Loop Closure Factor Integration

When STD detects a loop between keyframe `current_id` and `matched_id`:

```cpp
// 1. Detect loop
std::pair<int, double> search_result(-1, 0);
std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
std_manager->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair);

if (search_result.first > 0) {
    int matched_frame_id = search_result.first;
    
    // 2. Refine transformation with geometric ICP
    std_manager->PlaneGeomrtricIcp(
        std_manager->plane_cloud_vec_.back(),
        std_manager->plane_cloud_vec_[matched_frame_id],
        loop_transform);
    
    // 3. Add loop closure factor to GTSAM graph
    addLoopClosureFactor(current_id, matched_frame_id, loop_transform);
}
```

#### Noise Model Configuration

**For Loop Closure Factors:**
```cpp
// Use robust noise model (Cauchy kernel) to handle outliers
double loopNoiseScore = 0.1;  // Adjust based on confidence
gtsam::Vector robustNoiseVector6(6);
robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
                      loopNoiseScore, loopNoiseScore, loopNoiseScore;

gtsam::noiseModel::Base::shared_ptr robustLoopNoise =
    gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
```

**Why Robust Noise Model?**
- Handles false loop detections gracefully
- Prevents catastrophic failures from incorrect loop closures
- Similar to LIO-SAM's approach for GPS factors

---

### Implementation Steps

#### Step 1: Add STD Manager to LIO-SAM

```cpp
// In mapOptimization.h
#include "path/to/STDesc.h"

class mapOptimization {
private:
    STDescManager* std_manager;
    ConfigSetting config_setting;
    
    // Storage for STD (independent frame counting)
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> accumulated_clouds_for_std;
    int std_frame_count;           // Frame counter for STD (independent of LIO-SAM)
    int std_keyframe_interval;     // e.g., 10 frames
    
    // Loop closure storage
    std::vector<std::pair<int, int>> loop_index_container;
    
    // Functions
    void initializeSTD();
    void processEveryFrameForSTD();  // Called every frame, not just LIO-SAM keyframes
    void processSTDLoopClosure();
    void addSTDLoopFactor(int current_id, int matched_id, 
                         const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform);
};
```

#### Step 2: Initialize STD Manager

```cpp
void mapOptimization::initializeSTD() {
    // Read STD parameters
    nh.param<int>("std_keyframe_interval", std_keyframe_interval, 10);
    read_parameters(nh, config_setting);
    
    // Create STD manager
    std_manager = new STDescManager(config_setting);
    std_frame_count = 0;
}
```

#### Step 3: Process Every Frame for STD (Key Change!)

**Critical:** Process STD on **every frame**, not just LIO-SAM keyframes.

```cpp
void mapOptimization::processEveryFrameForSTD() {
    // This should be called in your main point cloud callback
    // BEFORE LIO-SAM's keyframe selection logic
    
    // Get current cloud in world frame
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*laserCloudIn, *world_cloud, transformTobeMapped);
    
    // Downsample for efficiency
    down_sampling_voxel(*world_cloud, 0.5);
    
    // Accumulate for STD (every frame, not just keyframes)
    accumulated_clouds_for_std.push_back(world_cloud);
    std_frame_count++;
    
    // Check if STD keyframe based on FRAME COUNT (not LIO-SAM's keyframe logic)
    if (std_frame_count >= std_keyframe_interval) {
        processSTDLoopClosure();
        std_frame_count = 0;
        accumulated_clouds_for_std.clear();
    }
}

// In your main point cloud callback:
void mapOptimization::laserCloudInfoHandler(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // ... existing LIO-SAM preprocessing ...
    
    // Process STD independently (every frame)
    processEveryFrameForSTD();
    
    // LIO-SAM's keyframe logic (distance/rotation based)
    if (saveFrame()) {
        saveKeyFramesAndFactor();  // LIO-SAM's keyframe processing
    }
}
```

#### Step 4: Process STD Loop Closure

```cpp
void mapOptimization::processSTDLoopClosure() {
    // Merge accumulated clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& cloud : accumulated_clouds_for_std) {
        *merged_cloud += *cloud;
    }
    
    // Downsample merged cloud
    down_sampling_voxel(*merged_cloud, config_setting.ds_size_);
    
    // Generate descriptors
    std::vector<STDesc> stds_vec;
    std_manager->GenerateSTDescs(merged_cloud, stds_vec);
    
    // Search for loops
    std::pair<int, double> search_result(-1, 0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
    
    int current_std_keyframe_id = std_manager->key_cloud_vec_.size();
    if (current_std_keyframe_id > config_setting.skip_near_num_) {
        std_manager->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair);
    }
    
    // Add to database
    std_manager->AddSTDescs(stds_vec);
    
    // If loop detected, add factor
    if (search_result.first >= 0) {
        int matched_std_id = search_result.first;
        
        // Refine with ICP
        std_manager->PlaneGeomrtricIcp(
            std_manager->plane_cloud_vec_.back(),
            std_manager->plane_cloud_vec_[matched_std_id],
            loop_transform);
        
        // Map STD keyframe ID to closest LIO-SAM keyframe ID
        // (since STD and LIO-SAM use different keyframe strategies)
        int current_liosam_id = cloudKeyPoses3D->size() - 1;
        int matched_liosam_id = findClosestLIOSAMKeyframe(matched_std_id);
        
        // Add loop closure factor
        addSTDLoopFactor(current_liosam_id, matched_liosam_id, loop_transform);
        
        // Store for visualization
        loop_index_container.push_back({current_liosam_id, matched_liosam_id});
    }
}
```

#### Step 5: Add Loop Factor to Graph

```cpp
void mapOptimization::addSTDLoopFactor(
    int current_id, int matched_id,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform) {
    
    // Get poses from LIO-SAM's pose graph
    gtsam::Pose3 pose_from = poseFrom(cloudKeyPoses6D->points[matched_id]);
    gtsam::Pose3 pose_to = poseFrom(cloudKeyPoses6D->points[current_id]);
    
    // Create relative transformation
    Eigen::Affine3d delta_T = Eigen::Affine3d::Identity();
    delta_T.translate(loop_transform.first);
    delta_T.rotate(loop_transform.second);
    
    gtsam::Pose3 pose_to_refined = gtsam::Pose3(delta_T.matrix()) * pose_to;
    
    // Compute relative pose
    gtsam::Pose3 relative_pose = pose_from.between(pose_to_refined);
    
    // Add factor with robust noise model
    double loopNoiseScore = 0.5;  // Can be tuned based on search_result.second
    gtsam::Vector robustNoiseVector6(6);
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
                          loopNoiseScore, loopNoiseScore, loopNoiseScore;
    
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise =
        gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    
    // Add to LIO-SAM's graph
    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        matched_id, current_id, relative_pose, robustLoopNoise));
    
    // Trigger optimization
    aLoopIsClosed = true;
}
```

---

### Code Examples

#### Complete Integration Example

See `demo/online_demo.cpp` in this repository for a complete working example of STD integration with FAST-LIO2. The integration pattern is similar for LIO-SAM.

**Key differences for LIO-SAM:**
1. STD processes **every frame** using frame-count-based keyframes
2. LIO-SAM uses **adaptive keyframes** based on distance/rotation
3. Two keyframe strategies operate **independently**
4. Need to map between STD keyframe indices and LIO-SAM keyframe indices
5. Use LIO-SAM's existing `gtSAMgraph` for loop factors

#### Parameter Tuning

**In your LIO-SAM config file:**
```yaml
# STD Parameters
std_keyframe_interval: 10        # Create STD keyframe every 10 frames (not LIO-SAM keyframes!)
ds_size: 0.5                     # Downsampling voxel size
descriptor_min_len: 2.0          # Minimum triangle side length
descriptor_max_len: 50.0         # Maximum triangle side length
skip_near_num: 50                # Skip recent N frames when searching
candidate_num: 50                # Number of candidates to consider
icp_threshold: 0.5               # Loop detection threshold
sub_frame_num: 10                # Frames to accumulate per keyframe (same as std_keyframe_interval)
```

---

### Best Practices

1. **Point Cloud Quality**: Ensure point clouds have sufficient geometric features
2. **Keyframe Interval**: Balance between computational cost and loop detection recall
3. **Noise Tuning**: Start with higher noise values (0.5-1.0) and tune down
4. **Verification**: Always use geometric verification (ICP) before adding loop factors
5. **Visualization**: Publish loop closure markers for debugging

---

## 中文

### 概述

本指南详细说明如何将 STD（稳定三角形描述符）回环检测模块集成到 LIO-SAM 中，通过鲁棒的回环检测和位姿图优化来提升 SLAM 性能。

### 目录
1. [点云数据要求](#点云数据要求-1)
2. [关键帧选择策略](#关键帧选择策略-1)
3. [因子图集成](#因子图集成-1)

---

### 点云数据要求

#### 应该传入什么样的点云给 STD？

将 STD 集成到 LIO-SAM 时，应该传入**全局/世界坐标系下的关键帧点云**给 STD 描述符生成器。

**输入要求：**
- **坐标系**：世界/全局坐标系（不是机体/传感器坐标系）
- **点云类型**：多帧扫描累积的点云（关键帧）
- **数据格式**：`pcl::PointCloud<pcl::PointXYZI>`
- **预处理**：运动畸变已校正（LIO-SAM 已提供）

**LIO-SAM 提供的数据：**
- /cloud_registered：全局坐标系下的点云（已完成运动补偿）
- /cloud_registered_body：机体坐标系下的点云
- **建议使用 /cloud_registered 或手动将 /cloud_registered_body 转换到世界坐标系**

#### 点云累积策略

STD 在**累积点云**上效果最好，需要累积多个连续帧。参考 online_demo.cpp 的实现：

```cpp
// 为关键帧累积点云
PointCloud::Ptr key_cloud(new PointCloud);

// 在达到关键帧阈值之前的每一帧
*key_cloud += *current_cloud_world;

// 当达到关键帧时（例如每 10 帧）
if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
    std_manager->GenerateSTDescs(key_cloud, stds_vec);
    key_cloud->clear(); // 为下一个关键帧重置
}
```

**LIO-SAM 集成建议：**
- 累积 10-20 帧（取决于 sub_frame_num 参数）
- 提供更丰富的几何信息用于描述符提取
- 确保有足够的角点特征用于场景识别

---

### 关键帧选择策略

**LIO-SAM 的原生关键帧选择**基于：
1. **距离阈值**：机器人移动超过 X 米时创建新关键帧
2. **旋转阈值**：机器人旋转超过 Y 度时创建新关键帧
3. **时间阈值**：关键帧之间的最小时间间隔

**STD 的关键帧策略**：
- 使用简单的**基于帧数的策略**（每 N 帧一个关键帧，例如 N=10）
- 累积 N 个连续帧形成一个关键帧点云

**推荐集成方案：使用 STD 的帧数策略（独立于 LIO-SAM 关键帧）**

关键要点是 STD 应使用**自己的基于帧数的关键帧选择**，独立于 LIO-SAM 的基于距离/旋转的关键帧选择。这意味着：

1. **累积每一帧**的世界坐标点云（来自 LIO-SAM 的里程计）
2. **独立计数帧数**（不依赖 LIO-SAM 的关键帧逻辑）
3. **每 N 帧创建 STD 关键帧**（例如 N=10）
4. **将累积的点云传递给 STD**

```cpp
// 在 LIO-SAM 的主循环中（处理每一帧，而不仅仅是 LIO-SAM 关键帧）
void processPointCloud() {
    // 从 LIO-SAM 获取当前世界坐标系点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*currentCloud, *world_cloud, currentPose);
    
    // 为 STD 累积（独立于 LIO-SAM 关键帧选择）
    accumulated_clouds_for_std.push_back(world_cloud);
    std_frame_count++;
    
    // STD 关键帧逻辑（每 N 帧，无论 LIO-SAM 关键帧如何）
    if (std_frame_count >= std_keyframe_interval) {
        // 合并累积的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto& cloud : accumulated_clouds_for_std) {
            *merged_cloud += *cloud;
        }
        
        // 生成 STD 描述符并搜索回环
        std_manager->GenerateSTDescs(merged_cloud, stds_vec);
        std_manager->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair);
        
        if (search_result.first >= 0) {
            addSTDLoopFactor(current_liosam_id, matched_liosam_id, loop_transform);
        }
        
        std_manager->AddSTDescs(stds_vec);
        
        // 为下一个 STD 关键帧重置
        accumulated_clouds_for_std.clear();
        std_frame_count = 0;
    }
}
```

**为什么采用这种方法？**
- STD 的帧数策略更简单、更可预测
- 将 STD 处理与 LIO-SAM 的自适应关键帧选择解耦
- 确保 STD 描述符的时间间隔一致
- 符合 STD 的设计理念

---

### 因子图集成

#### 回环因子添加

当 STD 检测到回环时，需要将回环约束作为因子添加到 LIO-SAM 的 GTSAM 因子图中：

```cpp
// 使用鲁棒噪声模型添加回环因子
double loopNoiseScore = 0.5;
gtsam::Vector robustNoiseVector6(6);
robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
                      loopNoiseScore, loopNoiseScore, loopNoiseScore;

gtsam::noiseModel::Base::shared_ptr robustLoopNoise =
    gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

// 添加到 LIO-SAM 的 gtSAMgraph
gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    matched_id, current_id, relative_pose, robustLoopNoise));

// 触发优化
aLoopIsClosed = true;
```

**关键要点：**
1. 使用鲁棒噪声模型（Cauchy 核）处理异常回环
2. 直接添加到 LIO-SAM 的 gtSAMgraph 中
3. 设置 aLoopIsClosed 标志触发位姿图优化
4. 在 STD 关键帧 ID 和 LIO-SAM 关键帧 ID 之间建立映射

---

### 参考资料

- [STD 论文](https://arxiv.org/abs/2209.12435)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- 完整代码示例：参见 demo/online_demo.cpp
