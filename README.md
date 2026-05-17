# STM32H503CB_LIS2DUX12
STM32H503CB_LIS2DUX12


# Overview
- **Name**:LIS2DUX12-V1.0
- **MCU**: STM32H503CBT6
- **IDE**: STM32CUBEMX+KEIL


# Buy Link
[https://shop192352884.taobao.com/](https://shop192352884.taobao.com/)




# Image

<img width="1339" height="1260" alt="image" src="https://github.com/user-attachments/assets/54b2699e-c15e-46df-9237-c6e8e9a926ec" />

<img width="1240" height="1760" alt="image" src="https://github.com/user-attachments/assets/2798984e-a45f-484a-9457-f74c7b9c503d" />

<img width="1241" height="1761" alt="image" src="https://github.com/user-attachments/assets/d3b0ae22-7dbf-492b-874c-491e4b73234a" />

<img width="1238" height="1759" alt="image" src="https://github.com/user-attachments/assets/d624cc15-edc1-4a5a-80c0-1d831631b210" />

<img width="1243" height="1763" alt="image" src="https://github.com/user-attachments/assets/653debff-9d30-49ac-9cdf-7e9bac0c5d9b" />





# Contact Information

- **Name**: Billy
- **交流群**: 925643491
- **Email**: a845656974@outlook.com
- **Phone**: +86 15622736378
- **CSDN Blog**: [Blog](https://blog.csdn.net/qq_24312945)
- **Video**: [Video](https://space.bilibili.com/26152390)



# Project Introduction
- **STM32H503CB_LIS2DUX12_Project1**:三轴加速度计LIS2DUX12开发(1)----轮询获取加速度数据
- **CSDN Blog**:[https://blog.csdn.net/qq_24312945/article/details/152116334](https://blog.csdn.net/qq_24312945/article/details/152116334)

本文将介绍如何驱动和利用LIS2DUX12传感器，实现精确的运动感应功能。
LIS2DUX12是一款数字式智能3轴线性加速度计，其MEMS和ASIC旨在将尽可能低的电流消耗与丰富的特性（如常开抗混叠滤波、有限状态机 (FSM)、具有自适应自配置 (ASC) 的机器学习内核 (MLC)）相结合。
FSM和MLC（带有ASC）为LIS2DUX12提供了始终可用的出色边缘处理能力。LIS2DUX12 MIPI I3C®从接口和嵌入式128级FIFO缓冲区构成了一系列功能，这让该加速度计在物料清单、处理能力和功耗上成为系统集成方面的参考。
LIS2DUX12具有±2g/±4g/±8g/±16g的用户可选满量程，并且可通过1.6 Hz到800 Hz的输出数据速率测量加速度。
LIS2DUX12包含专用内部引擎，用于处理运动和加速度检测，包括自由落体、唤醒、单/双/三击识别、活动/休止，以及6D/4D方向。
LIS2DUX12采用纤薄的小型塑料平面网格阵列封装(LGA)，可确保在更大的温度范围（-40°C至+85°C）内正常工作。
This article will introduce how to drive and use the LIS2DUX12 sensor to achieve accurate motion-sensing functions.

The LIS2DUX12 is a digital smart 3-axis linear accelerometer. Its MEMS and ASIC are designed to combine the lowest possible current consumption with rich features, such as always-on anti-aliasing filtering, a finite state machine (FSM), and a machine learning core (MLC) with adaptive self-configuration (ASC).

The FSM and MLC with ASC provide the LIS2DUX12 with excellent always-available edge processing capability. The LIS2DUX12 MIPI I3C® slave interface and embedded 128-level FIFO buffer form a comprehensive feature set, making this accelerometer a reference solution for system integration in terms of bill of materials, processing capability, and power consumption.

The LIS2DUX12 offers user-selectable full-scale ranges of ±2g, ±4g, ±8g, and ±16g, and can measure acceleration with output data rates from 1.6 Hz to 800 Hz.

The LIS2DUX12 includes dedicated internal engines for motion and acceleration detection, including free-fall detection, wake-up detection, single/double/triple-tap recognition, activity/inactivity detection, and 6D/4D orientation detection.

The LIS2DUX12 is available in a thin, small plastic land grid array package (LGA), ensuring operation over an extended temperature range from -40°C to +85°C.



- **STM32H503CB_LIS2DUX12_Project2**:加速度计LIS2DUX12开发(2)----静态校准
- **CSDN Blog**:[https://blog.csdn.net/qq_24312945/article/details/158503414](https://blog.csdn.net/qq_24312945/article/details/158503414)

零偏是影响加速度计输出精度的重要指标之一，零偏可分为静态零偏和动态零偏 。静态零偏也称为固定零偏，通常经标定与补偿减小静态零偏。动态零偏是由于加速度计自身的缺陷或环境因素（如温度、振动、电子干扰等）引起的，悬丝加速度计在运动过程中其精度会受到动态零偏的影响，因此在投入使用前要先对加速度计的动态零偏进行测试。
Zero offset is one of the important indicators affecting the output accuracy of an accelerometer. Zero offset can be divided into static zero offset and dynamic zero offset.

Static zero offset is also known as fixed zero offset, and it is usually reduced through calibration and compensation.

Dynamic zero offset is caused by the accelerometer’s own defects or environmental factors, such as temperature, vibration, and electronic interference. For a pendulous accelerometer, its accuracy during motion is affected by dynamic zero offset. Therefore, before the accelerometer is put into use, its dynamic zero offset should be tested first.























