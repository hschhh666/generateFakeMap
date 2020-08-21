# 程序名
generateFakeMap

# 功能
仿真行人运动，并输出仿真过程的视频和生成的行人轨迹地图

# 输入
配置文件，文件名为*.ini

# 输出
1. (可选)仿真过程的视频
1. (可选)行人轨迹地图，包括yml文件（即原始数据，用于后续计算）和png文件（yml文件的可视化）

# 使用方法
工程编译后生成generateFakeMap.exe可执行文件，执行命令
```
./generateFakeMap.exe simulator.ini
```
即可运行

# 配置文件
## 配置文件示例
```
[param]
visualRate = 0 # 仿真过程的播放速度，如果设置为0则不在线播放仿真过程
simulationTime = 120 # 仿真时长，单位为秒
simulationNum = 1 # 在当前配置下仿真几次
baseName = SouthEast_M1_P # 生成地图的文件名


[input]
sceneFile = D:\Research\IV2020MapPrediction\Code\MapPrediction\GenerateFakeMap\datas\satelliteMap\SEMask.yml # 场景原始文件，描述出入口、墙
satelliteFile = D:\Research\IV2020MapPrediction\Code\MapPrediction\GenerateFakeMap\datas\satelliteMap\SESatellite.png # 场景的卫星地图，只做可视化用
pedestrianMatrix = D:\Research\IV2020MapPrediction\Code\MapPrediction\GenerateFakeMap\datas\satelliteMap\SETestMatrix.yml # 行人在出入口生成的速度矩阵

[output]
video = D:/Research/IV2020MapPrediction/Code/MapPrediction/GenerateFakeMap/tmp.avi #仿真过程视频的保存路径，如果该参数为空则不保存视频
stateMapDir = D:\Research\IV2020MapPrediction\Code\MapPrediction\GenerateFakeMap/  #生成的状态地图（yml和png）的保存路径
```

## 配置文件说明
1. [input] 指程序运行时需要从外部读入的数据，总共有三个，分别为`sceneFile`、`satelliteFile`和`pedestrainMatrix`。其中
    1. `sceneFile`由另一个程序`generateScene`生成，描述了场景的出入口和障碍物，为opencv的yml文件，其数据结构参看`generateScene`源代码；
    1. `satelliteFile`为png文件，是自己从谷歌地图上截图截出来的，只是为了可视化使用。需要注意卫星图png与场景图结构图yml的大小是一样的，就相当于yml文件是卫星图png的mask，卫星图中的障碍物、出口就在yml中呈现出来。
    1. `pedestrainMatrix` 非常重要，**它直接决定了场景的动态状态**，它本质上是一个nxn的矩阵，储存为opencv的yml数据格式。假设某个场景一共有6个出入口，那么该文件就是6x6的矩阵，其中第`i`行`j`列描述的是**平均每分钟**从位置`i`走向位置`j`的人数。该文件的生成方式就随意了，自己写个python脚本就行。

2. [param] 程序运行时需要的一些内部参数。
    1. `visualRate`，如注释，不再赘述
    1. `simulationTime`，某次仿真时长，是仿真60秒还是120秒还是180秒等等
    1. `simulationNum`，在当前配置下仿真几次（仿真几次就生成几张地图）。这个参数主要是为了计算data variance的，如果只是做实验的话设置为1，即在当前配置下仿真一次就行了。
    1. `baseName`，输出的地图的名字，之所以叫'base name'，其实也是为了配合`simulationNum`的。比如我给`baseName`起的一个名字叫`SouthEast_M1_P`，此时如果设置`simulationNum=3`的话，最后在`stateMapDir`路径下生成的3张地图名字分别为`SouthEast_M1_P0`、`SouthEast_M1_P1`和`SouthEast_M1_P2`，意思是东南门在行人状态为`pedestrainMatrix = M1`的第0次、1次和2次仿真结果。

3. [output]注释说得很清楚了，不再赘述


# 使用流程
我们想做的是在某个场景仿真不同的行人动态模式，所以`sceneFile`、`satelliteFile`给定后，通过在不同的 `pedestrainMatrix`下仿真就可以得到该场景的不同动态模式。

以东南门为例，我在仿真时的方法是这样的：
1. 在google map上截图出东南门的卫星图，需要注意图片的size（即图像长宽）和pixel size（每个像素代表多少米）
1. 利用`generateScene`，配合卫星地图，标出该场景的mask，即障碍无区和出入口，生成yml文件，描述了场景的结构
1. 生成`n`个`pedestrainMatrix`，其中第`i`个`pedestrainMatrix`表示的就是第`i`分钟东南门的行人进出状况。其实吧，整个程序的核心控制数据就是`pedestrainMatrix`，`pedestrainMatrix`在时间上的变化，反映到仿真结果上就是动态状态在时间上发生变化。至于`pedestrainMatrix`如何生成那就很灵活了。
1. 生成n个配置文件`simulator_j.ini`，这n个配置文件的区别就在于`pedestrainMatrix`。运行n次`./generateFakeMap.exe simulator_j.ini`，得到n个在不同`pedestrainMatrix`下的状态地图。

# 其他说明
1. 我已经做好了东门和东南门的卫星图及结构图`satelliteFile`、`sceneFile`，直接用就行，不用再生成了。主要就是要思考如何生成`pedestrainMatrix`。在论文里，我是用两个变量`PN`和`FD`控制`pedestrainMatrix`的生成的，当然更复杂的生成方式会有更贴合实际的动态状态。
2. 程序输出的yml文件（即场景的动态状态）是用于后续深度学习计算的，而png文件只是对yml文件的可视化，完全不影响深度学习运算，是给人看的。这版程序中使用的可视化方式是对四个方向分别可视化，再可视化整体的人流量，但是这种可视化方式并没有被采用到论文的最终版中。论文最终版的可视化方式是把不同方向的人流量按不同颜色可视化到同一张图上，这是通过一个名叫`visStateMap.py`的python脚本实现的，该脚本输入为yml文件，输入为可视化的png文件。我实在懒得再改这个工程里的可视化代码了，python转C++好烦人，所以就这么用吧。生成yml后，再用`visStateMap.py`生成一下可视化的png。