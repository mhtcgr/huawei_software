#include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

class Cargo{
public:
    int value;
    int x, y, xb, yb; //xb,yb为目标泊位的坐标
    int time;
    Cargo():xb(-1),yb(-1),time(-1){}; //初始化泊位坐标为（-1，-1, 可存在时间为-1

    int findBerth();
}cargos[N][N];

class Robot{
public:
    int id;
    int cargoValue;
    int x, y;
    int carryState;
    int targetBerth;
    int targetCargo;
    int movingState;

    //一些初始化
    Robot():id(-1),cargoValue(0),carryState(0),targetBerth(-1),targetCargo(-1),movingState(1) {}

    void collisionDetection();

    void move();

    int plan();

    void get();

    void pull();

    int findCargo();

}robots[robot_num + 10];

class Berth{
public:
    int id;
    int x;
    int y;
    int boats[5]={-1,-1,-1,-1,-1}; //无船则为-1
    int transport_time;
    int loading_speed;
    int cargoNum;
    Berth():cargoNum(0){};

}berths[berth_num + 10];

class Boat{
public:
    int capacity;
    int cargoNum;
    int id;
    int targetBerth;
    int state;
    Boat():cargoNum(0),targetBerth(-2),state(1){}; //目标泊位为-2表示当前为初始状态，无目标(因为-1表示虚拟点）

    void ship();

    void go();
}boats[10];

class Gird{
public:
    char type;
    int robotId;
    Gird():robotId(-1){}; //-1表示无机器人

}Map[n][n];


int money, boat_capacity, id;

void Init()
{
    char line[N];
    for(int i = 0; i < n; i ++){
        scanf("%s", line); //地图输入
        //std::printf(line);
        for(int j=0;j<n;j++){
            switch (line[j]) {
                case 'B':{
                    Map[i][j].type='B';
                    break;
                }
                case 'A':{
                    Map[i][j].type='A';
                    break;
                }
                case '.':{
                    Map[i][j].type='.';
                    break;
                }
                case '*':{
                    Map[i][j].type='*';
                    break;
                }
                case '#':{
                    Map[i][j].type='#';
                    break;
                }

                default:break;

            }
        }
    }
        
        
    for(int i = 0; i < berth_num; i ++)
    {
        int id; //港口id
        scanf("%d", &id);
        berths[id].id=id;
        //港口的坐标，运输到虚拟点的时间，装载速度
        scanf("%d%d%d%d", &berths[id].x, &berths[id].y, &berths[id].transport_time, &berths[id].loading_speed);
    }

    //初始化船的属性
    scanf("%d", &boat_capacity); //船的容量
    for(int i=0;i<5;i++){
        boats[i].capacity=boat_capacity;
        boats[i].id=i;
    }

    //初始化机器人的id
    for(int i =0;i<10;i++){
        robots[i].id=i;
    }
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

int Input(){
    scanf("%d%d", &id, &money); //帧序号，当前金钱数

    int num; //新增的货物量
    scanf("%d", &num);
    for(int i = 0; i < num; i ++)
    {
        int x,y,v;
        scanf("%d%d%d", &x,&y,&v);  //货物的坐标和金额
        cargos[x][y].x=x;
        cargos[x][y].y=y;
        cargos[x][y].value=v;
        cargos[x][y].time=1000;
    }

    //这里需要遍历货物，生存时间减一，找泊位

    //接下来10行robot数据
    for(int i = 0; i < robot_num; i ++)
    {
        //是否携带物品，坐标，状态（恢复状态还是正常状态）
        scanf("%d%d%d%d", &robots[i].carryState, &robots[i].x, &robots[i].y, &robots[i].movingState);
        //把机器人的id标注到地图上
        Map[robots[i].x][robots[i].y].robotId=i;
    }
    
    //接下来5行boat数据
    for(int i = 0; i < 5; i ++)
        //船的状态（运输中，正常状态，泊位外等待状态），目标泊位
        scanf("%d%d\n", &boats[i].state, &boats[i].targetBerth);

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
        for(int i = 0; i < robot_num; i ++)
            printf("move %d %d\n", i, rand() % 4);
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
