classdef VideoStab < handle
    properties
        smoothedMat
        k
        errscaleX
        errscaleY
        errthetha
        errtransX
        errtransY
        
        Q_scaleX
        Q_scaleY
        Q_thetha
        Q_transX
        Q_transY
        
        R_scaleX
        R_scaleY
        R_thetha
        R_transX
        R_transY
        
        sum_scaleX
        sum_scaleY
        sum_thetha
        sum_transX
        sum_transY
        
        scaleX
        scaleY
        thetha
        transX
        transY
        
        HORIZONTAL_BORDER_CROP = 20
    end
    
    methods
        function obj = VideoStab()
            % Kalman filter parameters
            Q1 = 0.004;
            R1 = 0.5;
            
            obj.smoothedMat = zeros(2, 3);
            obj.k = 1;
            
            obj.errscaleX = 1;
            obj.errscaleY = 1;
            obj.errthetha = 1;
            obj.errtransX = 1;
            obj.errtransY = 1;
            
            obj.Q_scaleX = Q1;
            obj.Q_scaleY = Q1;
            obj.Q_thetha = Q1;
            obj.Q_transX = Q1;
            obj.Q_transY = Q1;
            
            obj.R_scaleX = R1;
            obj.R_scaleY = R1;
            obj.R_thetha = R1;
            obj.R_transX = R1;
            obj.R_transY = R1;
            
            obj.sum_scaleX = 0;
            obj.sum_scaleY = 0;
            obj.sum_thetha = 0;
            obj.sum_transX = 0;
            obj.sum_transY = 0;
            
            obj.scaleX = 0;
            obj.scaleY = 0;
            obj.thetha = 0;
            obj.transX = 0;
            obj.transY = 0;
        end
        
        function smoothedFrame = stabilize(obj, frame_1, frame_2)
            % 转灰度
            frame1 = im2gray(frame_1);
            frame2 = im2gray(frame_2);
            
            vert_border = round(obj.HORIZONTAL_BORDER_CROP * size(frame_1,1) / size(frame_1,2));
            
            % 特征点检测 + 光流跟踪
            points1 = detectMinEigenFeatures(frame1,'MinQuality',0.01,'FilterSize',7);
            features1 = points1.Location;
            
            tracker = vision.PointTracker('MaxBidirectionalError', 3);
            initialize(tracker, features1, frame1);
            [features2, valid] = step(tracker, frame2);
            
            goodFeatures1 = features1(valid,:);
            goodFeatures2 = features2(valid,:);
            
            % 估计 4 自由度仿射变换，scale,theta,tx,ty
            tform = estgeotform2d(goodFeatures1,goodFeatures2,"similarity");
            affine = tform.A(1:2,:);
            
            dx = affine(1,3);
            dy = affine(2,3);
            da = atan2(affine(2,1), affine(1,1)); % 弧度
            ds_x = affine(1,1) / cos(da);
            ds_y = affine(2,2) / cos(da);
            
            sx = ds_x;
            sy = ds_y;
            
            obj.sum_transX = obj.sum_transX + dx;
            obj.sum_transY = obj.sum_transY + dy;
            obj.sum_thetha = obj.sum_thetha + da;
            obj.sum_scaleX = obj.sum_scaleX + ds_x;
            obj.sum_scaleY = obj.sum_scaleY + ds_y;
            
            % 第一次不做预测
            if obj.k == 1
                obj.k = obj.k + 1;
            else
                obj.Kalman_Filter();
            end
            
            % Compute differences between smoothed and raw accumulated parameters.
            diff_scaleX = obj.scaleX - obj.sum_scaleX;
            diff_scaleY = obj.scaleY - obj.sum_scaleY;
            diff_transX = obj.transX - obj.sum_transX;
            diff_transY = obj.transY - obj.sum_transY;
            diff_thetha = obj.thetha - obj.sum_thetha;
            
            ds_x = ds_x + diff_scaleX;
            ds_y = ds_y + diff_scaleY;
            dx = dx + diff_transX;
            dy = dy + diff_transY;
            da = da + diff_thetha;

           
            % 平滑后的仿射矩阵
            obj.smoothedMat(1,1) = sx * cos(da);
            obj.smoothedMat(1,2) = sx * -sin(da);
            obj.smoothedMat(2,1) = sy * sin(da);
            obj.smoothedMat(2,2) = sy * cos(da);
            obj.smoothedMat(1,3) = dx;
            obj.smoothedMat(2,3) = dy;
            
            % 相似变换
            tformSmooth = simtform2d([obj.smoothedMat; 0 0 1]);
            smoothedFrame = imwarp(frame_1, tformSmooth, 'OutputView', imref2d(size(frame_2)));
            
            % 裁剪黑边
            smoothedFrame = smoothedFrame(vert_border+1:end-vert_border, ...
                                          obj.HORIZONTAL_BORDER_CROP+1:end-obj.HORIZONTAL_BORDER_CROP,:);
        end
        
        function Kalman_Filter(obj)
            frame_1_scaleX = obj.scaleX;
            frame_1_scaleY = obj.scaleY;
            frame_1_thetha = obj.thetha;
            frame_1_transX = obj.transX;
            frame_1_transY = obj.transY;
            
            % 预测误差协方差: Add process noise Q (simple prediction step without motion model).
            frame_1_errscaleX = obj.errscaleX + obj.Q_scaleX;
            frame_1_errscaleY = obj.errscaleY + obj.Q_scaleY;
            frame_1_errthetha = obj.errthetha + obj.Q_thetha;
            frame_1_errtransX = obj.errtransX + obj.Q_transX;
            frame_1_errtransY = obj.errtransY + obj.Q_transY;
            
            % 计算卡尔曼增益: K = P / (P + R), where P is predicted covariance, R is measurement noise
            gain_scaleX = frame_1_errscaleX / (frame_1_errscaleX + obj.R_scaleX);
            gain_scaleY = frame_1_errscaleY / (frame_1_errscaleY + obj.R_scaleY);
            gain_thetha = frame_1_errthetha / (frame_1_errthetha + obj.R_thetha);
            gain_transX = frame_1_errtransX / (frame_1_errtransX + obj.R_transX);
            gain_transY = frame_1_errtransY / (frame_1_errtransY + obj.R_transY);
            
            % 更新状态估计: new_state = predicted + K * (measurement - predicted)
            % Measurements are the raw accumulated sums (obj.sum_xxx).
            obj.scaleX = frame_1_scaleX + gain_scaleX * (obj.sum_scaleX - frame_1_scaleX);
            obj.scaleY = frame_1_scaleY + gain_scaleY * (obj.sum_scaleY - frame_1_scaleY);
            obj.thetha = frame_1_thetha + gain_thetha * (obj.sum_thetha - frame_1_thetha);
            obj.transX = frame_1_transX + gain_transX * (obj.sum_transX - frame_1_transX);
            obj.transY = frame_1_transY + gain_transY * (obj.sum_transY - frame_1_transY);
            
            % 更新误差协方差矩阵 new_P = (1 - K) * predicted_P
            obj.errscaleX = (1 - gain_scaleX) * frame_1_errscaleX;
            obj.errscaleY = (1 - gain_scaleY) * frame_1_errscaleY;
            obj.errthetha = (1 - gain_thetha) * frame_1_errthetha;
            obj.errtransX = (1 - gain_transX) * frame_1_errtransX;
            obj.errtransY = (1 - gain_transY) * frame_1_errtransY;
        end
    end
end
