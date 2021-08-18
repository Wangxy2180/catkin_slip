[toc]

# 使用方法

1. 所有可调参数在launch文件中的注释中已经标出。
2. 先设定roslaunch中ROI区域，celex的坐标似乎是左下角是原点，所以设定时一定要小心，可以查找`cv::imshow("ROI_show",mat_ROI_);`，把这两句放开，你就能看到ROI区域的可视化图像了。
3. 在一切都设定好之后，找到roslaunch中的`isInitThtesholdTest`参数，设为`true`，`cor_init_threshold`和`event_init_threshold`设置为0，然后运行roslaunch，你会看到屏幕在疯狂输出`avg_event_num`和`avg_cor_num`，待输出的两个数值不再大幅变动后，我们认为他对环境的噪声采样趋于稳定，然后将这两个值乘一个缩小系数(我的是`0.6*avg_event_num`，`0.7*avg_cor_num`)，然后将该值再设定为对应的`event_init_threshold`和`cor_init_threshold`。采样完成后记得把`isInitThtesholdTest`设为`false`。(这一步是为了避免初始阶段事件变化幅度较大，容易造成阈值波动，产生误检测，所以先预先采样环境阈值，超出这个阈值了，才认为这个值有效，可以被用作阈值动态更新；而角点数量阈值因为仅在初始化时才计算阈值，后续没有滚动更新，所以这个缩小系数可以略微大一点)。（这一步还是有点用的，因为运行过程中也可能会出现时间突然降低，直接扔掉前五秒的数据并不能保证稳定。）
4. 看到屏幕输出`corner_threshold init done: xxx`之后，表示初始化完成，此时才可以进行检测，大概8s左右
5. 后续可以通过调整两个阈值缩放参数`dynamic_threshold_scale`和`cor_threshold_scale`调整他的敏感程度。但其实还是不可能完全避免误检，只能尽可能的减少。(可以将`EventCntSlipDetector::isSlipped()`中的那句`ROS_INFO("more--")`打开，这句上边是事件数量的检测，下边是角点数量的检测，可以判断有多少误检在第一步被拦截了，有多少误检又是在第二步被拦截的。这样就可以分开调整两个缩放参数。如果只有`more--`，则说明被第二部拦截了，如果出现了`more--`和`SLIPPPPPPPPPPPING`则说明这两步都认为他是产生了滑动，没有进行拦截)
6. 全部调整完成后直接运行launch文件就行，出现`corner_threshold init done: xxx`表示初始化完成，可以正常进行检测了。

160 240

- 订阅`/celex5_slip_detector/slip_signal`，收到字符串`SLIPPED`，表示滑动了。

- 一些可以调节的参数(在roslaunch中)

  - `dynamic_threshold_scale` 阈值缩放参数
  - `cor_threshold_scale` 角点阈值缩放参数，角点数量是在一开始初始化时设定的，在1280*800下，在我的实验环境下，角点数量大致在200~450之间浮动，如果出现运动物体，就会上千，因为角点检测并不是每个event batch都会执行，所以还没法做滚动更新。
  - `max_slip_cnt`，为了防止爪子无限制的去施加力设定，如果连续产生的滑动次数超过这个值，那么新的连续滑动就不会publish，当然，如果没有机械臂用力抓的这个动作，那就调大一点(至少要大于20，以防万一，建议再加几个0)。
  - `event_frame_time`，生成事件帧的事件
  - `ROI`区域：celex的数据要做一些翻转才能和人眼看到的一样，一定要小心。但其实这个似乎没有什么必要，我试了一下，在25cm的距离下，看到的大概是10.5*19cm的大小
  - `cor_init_threshold`和`event_init_threshold`对应两个update函数中的第一个if中的那个数，即超过这个阈值，我才认为这个点有效，可以用来更新。可以根据场景跑一下，看看静态场景下角点和事件的数量大致是多少，我的环境下，事件数量大概在420左右稳定，角点数量在230左右稳定，然后我给他乘了0.6和0.7，然后根据喜好简单调整一下，得到初始化时的最低数量阈值240 和160。

- 一些可以调节的参数(在源码中)

  - envWindowSize 保存事件数量的窗口的大小，现在是11，前10 用来计算阈值，第11个存储最新的事件数量

# 各种检测方法

**当前只建议使用cb模式下的Event检测方法**

- **Event_Off_Pixel_Timstamp_Mode**(1000Hz)：仅通过事件激增判断，再fast角点判断角点数量是否激增
- Optical_Flow_Mode(50Hz)：通过光流方向(down)以及事件量判断，再霍夫判断是否出现边缘
- Loop_Mode(50Hz 还没写好)：我想的是通过事件数量快速判断，然后通过光流细化，但是似乎没啥卵用

**Event_Off_Pixel_Timstamp_Mode** 现在只建议使用这个模式，别的速度达不到，且后续没有更新

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

- [ ] 为什么！！！为什么！！！一句小小的if，居然要20ms！！！5ms vs 25ms!!! 不加if(3~4ms)，加了if(24-26ms)，把两个大if搬过来是17-23ms。直接替换for中的值是8 ~10ms，把那两个值的计算过程省略，就正常3 4ms了

- [x] 总是会在最开始的时候检测到一个莫名其妙的滑动，可能还是对事件和角点更新阈值多次取值求平均更合适一点

    > 添加初始化阈值，小于该阈值的一律不采用

- [x] Loop_Mode和Optical_Flow_Mode下，连续产生大量事件，就会`generate_image: buffer is full!`，还未解决，不过普通的滑动检测场景可能也遇不到。

    > 不管了，反正用不上这两个模式了

- [x] 不知道为啥，明明设置了光流的帧时间，但是频率一直都是50Hz左右，似乎设置完没有起作用

	>因为范围是(10,180), cb:20：47Hz；11：56Hz ，主被动都会buffer is full，再试一试

- [x] 主动获取模式，`Event_Off_Pixel_Timestamp_Mode`下，事件一多，绝对会崩，其他不保证。

    >现在不是问题了，只建议使用cb模式

- [ ] FPN的path还没设置，不过问题不大，似乎也用不到这玩意。

- [x] `Event_Off_Pixel_Timstamp_Mode`太敏感了，可能会一下产生好多滑动信号，也许需要再判断一下。

- [x] 由于我的实验环境有限，无法完美模拟滑动场景，有些参数可能不太合适。阈值的计算方式在`updateEventWindow`中，用大小为10的滚动窗口的事件数量均值乘一个缩放参数(当前设置2.5)。

    >为方便调整，把参数写在roslaunch中了

- [ ] 时间戳计算出问题了，获取的当前包的外部时间戳总是比now快

  >不管他了，就这么着了，现在只计算检测所耗时间

- [x] hough变换太费时间了，不加，1ms内解决，加了基本上10ms 当然，这只是在我的电脑上，

  > 改成检测fast角点了，但是似乎没什么卵用，事件数量超出阈值，角点数量基本就一定会超出阈值

- [x] 对于连续滑动，要不要改成只通知一次

  >加了一个阈值，已经改了





# 一些注意事项

1. 文中有两个“123123123”的publish，这是为了测试运行频率设定的，可以直接把这两个部分注释掉
2. 记得那三个XML文件要复制过去



# 没了

eventCntSize + cb + 1000us

OFDirection + cb + 1ms

loop + cb+1ms buffer is full













