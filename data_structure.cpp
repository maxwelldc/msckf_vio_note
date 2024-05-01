namespace msckf_vio
{
    dataStructure:
        struct IMUState
        {id time orientation position velocity gyroBias accBias R_imu_cam t_imu_cam gyroNoise accNoise 
        gyroBias_Noise accBias_Noise gravity }

        struct CAMState
        {id time orientation position T_cam0_cam1}

        struct Feature
        {
            struct optimizationConfig{}
        }

    class MsckfVio
    {
        struct StateServer{IMUState CAMState stateCov}

        function:
        initialize()
        {
            1.loadParameter()：
            2.creatROSIO():发布：里程计、地图点云、reset  订阅：imu数据、图像处理节点
        }

        imuCallback()
        {
            1.save the imu data in a buffer 
            2.initializeGravityAndBias()
        }

        featureCallback()
        {
            1.判断是否是第一帧
            2.batchImuProcessing():IMU的卡尔曼预测过程（也就是误差状态传递）
            {
                processModel()
                {
                    1.去掉bias
                    2.状态转移矩阵F离散化
                    3.数值积分传递误差状态:predictNewState()
                    4.根据可观性约束得到phi矩阵，计算噪声的方差，进而计算全部的协方差矩阵
                }
            }
            3.stateAugmentation()：状态增广。做了两部分工作：
              1）根据已知的imu与相机外参以及Imu运动模型推测出当前相机的位姿并加入msckf状态向量中
              2）计算出增光状态对已有状态的雅克比，并对系统的协方差矩阵进行增广
            4.addFeatureObservations():判断是否有新的地图点，并加入地图中
            5.removeLostFeatures():
              1）首先标记无效的特征点，将特征点移除后记录特征点。
              2）计算特征点对于相机状态的雅克比矩阵。
              3）卡方检验后执行测量更新。将叠放在一起之后的雅克比矩阵分解，然后计算卡尔曼增益，更新所有状态和所有协方差
            6.pruneCamStateBuffer():将处理过的相机状态移除状态向量和协方差矩阵，
                                   计算移除状态对于其他状态的雅克比矩阵，
                                    执行一步测量更新
        }
        
        resetCallback()
    }

    class imageprocessor
    {
        data structure:
        struct ProcessorConfig {grid_row grid_col grid_min_feature_num grid_max_feature_num pyramid_levels;
     patch_size;fast_threshold;max_iteration;track_precision;ransac_threshold;stereo_threshold;}

        struct FeatureMetaData {id;response;lifetime;cam0_point;cam1_point;}

        function:
        1.creatImagePyramids()
        2.initializeFirstFrame(){detector_ptr->detect(img, new_features);}:提取特征点，保存特征点，划分格子
        3.drawFeaturesStereo():画特征点
        4.trackFeature()
        {
            1.integrateIMU():获得相机之间的旋转
            2.predictFeature():根据IMU的数据和单应性原理，预测下一帧关键点的位置
            3.LK光流跟踪
            4.外点剔除
        }
        5.addNewFeatures():将已经检测的特征点滤掉，重新检测特征点，并分配到格子里，多的删掉，少的增补
        6.drawFeaturesStereo():将结果画上
    }
}

前端：1.采用计算LK光流的方式不用计算描述子，节省时间
2.采用本质矩阵约束剔除外点
3.采用单应矩阵约束匹配下一个特征点
4.相机与IMU之间的联系在特征追踪。IMU在两帧图像之间视为匀速运动，将角度除以帧数得到平均速度，转化到相机坐标系，从相机的帧间时间得到旋转
角度（表现出紧耦合的特性）。从两帧之间的旋转和相机内参（单应性原理）得到下一帧的特征点位置，LK光流跟踪。

后端：1.采用ESKF，误差是状态（真值）和bias（加速度和加速度的bias）的差
2.F矩阵是通过误差状态量与误差状态量之间的导数的传递关系的出来的，G矩阵是通过误差状态量的导数和噪声量的出来的
3.误差状态的传递是通过预积分（四阶龙格库塔方法）实现的，得到新一时刻的pvq，不是通过状态方程实现的。
4.F矩阵是用来算phi矩阵的，phi矩阵和Q矩阵和G矩阵算Qk矩阵，phi和Qk来计算IMU的状态协方差Pii

需要继续研究的问题：
1.可观性分析以及FEJ(first estimate jacobian)
2.单应性问题
3.四元数动力学