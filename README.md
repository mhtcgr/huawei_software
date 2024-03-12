# HuaweiCompetition
# 3/7 15:43  
上传了初始的main文件  
# 3/11 11：09
完成了各种类的初稿，并初次实现了输入数据的保存
# 3/11 17：00
简单实现了boat和berth的运行（未调试过）
## Robot class
```
int id;
int cargoValue;
int x, y;
int carryState;
int targetBerth;
pair<int,int> targetCargo;
int movingState;
```
```c++
pair<int,int> Robot::findCargo();
//结合货物价格、货物到泊位的距离和自身到货物的距离来综合选择
```

```c++
Point findNearestBerthGrid(const Point& robot, const Point& berth);
//找到离机器人最近的泊位距离机器人最近的一个格子
```

```c++
void Robot::plan();
```

> 首先，如果机器人carryState==0，那就先调用findCargo()，反之，就可以直接走向泊位。
>
> 一帧分为移动前动作、移动动作、移动后动作，所以先判定移动前动作。
>
> 是否需要get或者pull？
>
> 若手上没东西，而且走到了**目标货物**的位置，就可以get；
>
> 手上有东西，而且走到了泊位的范围，就可以pull；
>
> 然后是移动动作，确定往上下左右哪个方向移动（假定现在不考虑停下来，或者说当四个方向都不满足的时候其实也会停下来。）
>
> 一开始本来想用A*算法来计算出到达目标的最短路径，这样下一步一定是最短路径上的一步（已考虑障碍物）；但是这样需要不小的复杂度（虽然我已经写完了而且不想删掉，那就先放在文件的astar类里面吧OVO）
>
> 于是改成了把货物的位置分类成在左上角、右上角、右下、左下四个方位，然后依次判断：如果最近的方向可以走，就走这个方向，如果不行的话绕路
>
> 然后最后，还需要判定一下是否需要get或者pull
>
> 就没了。（尊嘟假嘟）
