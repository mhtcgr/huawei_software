#include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

class Robot {
public:
    int x, y, goods;//goods表示是否携带物品
    int status;
    int mbx, mby;//下一步的位置

    Robot() {}
    Robot(int startX, int startY):x(startX),y(startY) {}
};
Robot robot[robot_num + 10];

class Good {
public:
    int x,y,val;
    int last_frame;//还剩多少帧
    Good() {}
    Good(int startX, int startY):x(startX),y(startY),last_frame(1000) {}
};
//建立一个以val为比较值的最大优先队列，从最大的开始，查询最近的没有拿物品的机器人，让这个
//机器人朝自己的这个方向运动。

class Berth {
public:
    int x;
    int y;
    int transport_time;
    int loading_speed;

    Berth(){}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this -> x = x;
        this -> y = y;
        this -> transport_time = transport_time;
        this -> loading_speed = loading_speed;
    }
};
Berth berth[berth_num + 10];
class Boat {
public:
    int num, pos, status;
};
Boat boat[10];

int money, boat_capacity, id;//id是帧序号（从 1 开始递增）
char ch[N][N];
int gds[N][N];
void Init()
{
    for(int i = 1; i <= n; i ++)
        scanf("%s", ch[i] + 1);
    for(int i = 0; i < berth_num; i ++)
    {//泊位
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

int Input()
{//输入
    scanf("%d%d", &id, &money);
    int num;//新增货物的数量
    scanf("%d", &num);
    for(int i = 1; i <= num; i ++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        //把这些值放到货物对象里面
    }
    for(int i = 0; i < robot_num; i ++)
    {
        int sts;//状态
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
    }
    for(int i = 0; i < 5; i ++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    char okk[100];
    scanf("%s", okk);
    return id;
}

int main()
{
    Init();
    for(int zhen = 1; zhen <= 15000; zhen ++)
    {
        int id = Input();
        for(int i = 0; i < robot_num; i ++) {
            //printf("move %d %d\n", i, rand() % 4);
            //输出机器人的动作
        }
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
