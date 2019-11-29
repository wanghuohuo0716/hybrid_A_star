# 本仓库是对混合A*算法的matlab复现
## 算法结果图
![](https://github.com/wanghuohuo0716/hybrid_A_star/blob/master/image/Parking.gif)
![](https://github.com/wanghuohuo0716/hybrid_A_star/blob/master/image/Parking2.gif)
![](https://github.com/wanghuohuo0716/hybrid_A_star/blob/master/image/Straight.gif)
## 如何使用
1.在matlab中直接运行EntryPoint.m文件即可，坐标的航向phi取值范围是[-pi,pi]

注意：
1.本仓库提供两个代码版本，其中without_comments分支是原作者的版本，并且修复了从左边直线行驶轨迹错误的bug(原因是VehicleCollisionCheck出错)。
2.master分支是本人根据源代码进行修改，添加注释方便理解，同时也修改了VechicleCollisionCheck的代码。
## 文件组织
![](https://github.com/wanghuohuo0716/hybrid_A_star/blob/master/image/Mind.png)

## 感谢[柳梦璃](https://zhuanlan.zhihu.com/p/40776683)为本仓库开源的源代码！
