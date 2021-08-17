[toc]



# 使用方法

- Event_Off_Pixel_Timstamp_Mode(1000Hz)：仅通过事件激增判断，再~~霍夫~~判断是否出现直线

- Optical_Flow_Mode(50Hz)：通过光流方向(down)以及事件量判断，再霍夫判断是否出现边缘

- Loop_Mode(50Hz 还没写好)：我想的是通过事件数量快速判断，然后通过光流细化，但是似乎没啥卵用

- 

- 订阅`/celex5_slip_detector/slip_signal`，收到字符串`SLIPPED`，表示滑动了。

- 

- 一些可以调节的参数(roslaunch)

  - dynamic_threshold_scale_ 阈值缩放参数
  - corner_threshold_scale_ 角点阈值缩放参数
  - (这个没用)cor_threshold_ 角点数量的阈值，这个阈值设为0肯定是不合适的，在1280*800下，在我的实验环境下，大致在200~450之间浮动，如果产生震动相机，就会上千，因为角点检测并不是每次都会执行，所以还没法做阈值窗口(或许我可以在init阶段做一个？)
  - envWindowSize 保存事件数量的窗口的大小
  
  

callback 方式，

Event_Off_Pixel_Timstamp_Mode

```bash
roslaunch slip_detector slip_detector.launch detector_type:=slip_detector_node_cb celex_mode:=Event_Off_Pixel_Timestamp_Mode
```

Optical_Flow_Mode

```
roslaunch slip_detector slip_detector.launch detector_type:=slip_detector_node_cb celex_mode:=Optical_Flow_Mode
```

Loop_Mode

```
roslaunch slip_detector slip_detector.launch detector_type:=slip_detector_node_cb celex_mode:=Loop_Mode
```

建议使用callback下的`Optical_Flow_Mode`和`Event_Off_Pixel_Timstamp_Mode`模式

或者直接去launch文件中修改参数。



若使用主动获取方式，可以将`detector_type:=slip_detector_node_cb`替换为`detector_type:=slip_detector_node`。**但是**不保证能用，`Event_Off_Pixel_Timestamp_Mode`绝对会崩，其他不一定。



# 存在的问题

- [ ] Loop_Mode和Optical_Flow_Mode下，连续产生大量事件，就会`generate_image: buffer is full!`，还未解决，不过普通的滑动检测场景可能也遇不到。

- [x] 不知道为啥，明明设置了光流的帧时间，但是频率一直都是50Hz左右，似乎设置完没有起作用

>因为范围是(10,180), cb:20：47Hz；11：56Hz ，主被动都会buffer is full，再试一试

- [ ] 主动获取模式，`Event_Off_Pixel_Timestamp_Mode`下，事件一多，绝对会崩，其他不保证。

- [ ] FPN的path还没设置，不过问题不大，似乎也用不到这玩意。

- [ ] `Event_Off_Pixel_Timstamp_Mode`太敏感了，可能会一下产生好多滑动信号，也许需要再判断一下。

- [ ] 由于我的实验环境有限，无法完美模拟滑动场景，有些参数可能不太合适。阈值的计算方式在`updateEventWindow`中，用大小为10的滚动窗口的事件数量均值乘一个缩放参数(当前设置2.5)。
- [ ] 时间戳计算出问题了，获取的当前包的外部时间戳总是比now快
- [ ] hough变换太费时间了，不加，1ms内解决，加了基本上10ms 当然，这只是在我的电脑上，改成fast角点了
- [ ] 对于连续滑动，要不要改成只通知一次





# 一些注意事项

1. 文中有两个“123123123”的publish，这是为了测试运行频率设定的，可以直接把这两个部分注释掉
2. 记得那三个XML文件要复制过去



# 没了

eventCntSize + cb + 1000us

OFDirection + cb + 1ms

loop + cb+1ms buffer is full













