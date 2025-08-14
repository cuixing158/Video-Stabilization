# Real-Time Video Stabilization Using Kalman Filter
[![View Video-Stabilization on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/181778-video-stabilization)
[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=cuixing158/Video-Stabilization&file=Example_videoStabilization.mlx)

本项目实现了基于 **光流估计** + **Kalman 滤波** 的实时视频防抖算法[^1]，能够在视频播放时同时显示**防抖前后对比**效果。核心算法来源于光流法计算相邻帧的平移、旋转、缩放参数，并通过 Kalman 滤波器进行平滑，最终生成防抖后的稳定视频序列。关于常规Kalman滤波算法原理理解可以参考我于2018年写的博客[^2],仅供类比参考。



https://github.com/user-attachments/assets/68a1abb5-bcd4-4b24-86e4-cae4d0d11abb

左边为抖动视频，右边为稳定视频


## Requirements

- MATLAB R2022b or later
- Computer Vision Toolbox™
- Image Processing Toolbox™

## Example

```matlab
% 输入视频路径
videoPath = "./data/videoShake.m4v";  % 抖动视频

% 读取视频
vReader = VideoReader(videoPath);

% 创建防抖对象
stab = VideoStab();

% 读取第一帧
if hasFrame(vReader)
    prevFrame = readFrame(vReader);
else
    error('视频为空');
end

frameCount = 0;
showObj = imshow([prevFrame,prevFrame]);
t = title(sprintf('Frame %d', frameCount));
while hasFrame(vReader)
    currFrame = readFrame(vReader);
    frameCount = frameCount + 1;

    % 调用 stabilize
    smoothedFrame = stab.stabilize(prevFrame, currFrame);

    % 拼接前后对比
    canvas = imtile({prevFrame,smoothedFrame},GridSize=[1,2]);

    % 如果分辨率太大可以缩放
    if size(canvas,2) > 1920
        scaleFactor = 1920 / size(canvas,2);
        canvas = imresize(canvas, scaleFactor);
    end

    % 显示
    showObj.CData = canvas;
    t.String = sprintf('Frame %d', frameCount);

    % 当前帧变为下一轮的 prevFrame
    prevFrame = currFrame;

    drawnow;
end
```

## References

[^1]: L. Kejriwal and I. Singh, "A Hybrid Filtering Approach of Digital Video Stabilization for UAV Using Kalman and Low Pass Filter," Procedia Computer Science, vol. 93, pp. 359-366, 2016, doi: 10.1016/j.procs.2016.07.221.

[^2]: [基于卡尔曼滤波算法在三维球轨迹中跟踪应用](https://blog.csdn.net/cuixing001/article/details/84203398)
