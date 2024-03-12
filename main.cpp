#include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

class Cargo{
public:
    int value;
    int x, y, idb; //xb,yb为目标泊位的坐标
    int time;
    Cargo():value(-1),x(-1),y(-1),idb(-1),time(-1){}; //初始化泊位坐标为（-1，-1, 可存在时间为-1
    
    int findBerth();
}

int Cargo::findBerth(){//evaluation function for each berth
    //parameter for each evaluating dimension
    int a = 1;//pace to berth
    int b = 1;//loading time
    int c = 1;//berth time
    int d = 1;//todo:shipnum
    int result = 0;
    int steps = 0;//pace to berth by robot plan
    int max = 0;
    int targetid = 0;
    for (size_t i = 0; i < berth_num; i++)
    {
        int loadtime = berths[i].cargoNum/berths[i].loading_speed;//todo:cargonum after steps

        if(loadtime <= steps){
            result = a*steps+c*berths[i].transport_time;
        }else{
            result = a*steps+b*(loadtime-steps)+c*berths[i].transport_time;
        }

        if(max < result){
            max = result;
            targetid = i;
        }
    }
    return targetid;
    
}

class Robot{
public:
    int id;
    int cargoValue;
    int x, y;
    int carryState;
    int targetBerth;
    int targetCargox;
    int targetCargoy;
    int movingState;

    //一些初始化
    Robot():id(-1),cargoValue(0),carryState(0),targetBerth(-1),targetCargo(-1),movingState(1) {}

    void collisionDetection();

    void move();

    int plan();

    void get();//cargo position

    void pull(int id);

    int findCargo();

}robots[robot_num + 10];

void Robot::get(){
    
    Map[this->targetCargox][this->targetCargoy].type = '.';

    this->cargoValue = cargos[this->targetCargox][this->targetCargoy].value;
    this->carryState = 1;
    this->targetBerth = cargos[this->targetCargox][this->targetCargoy]->idb;

    delete cargos[this->targetCargox][this->targetCargoy];

}

void Robot::pull(int id){

    berths[id].cargoNum++;

    this->cargoValue = 0;
    this->carryState = 0;
    this->targetBerth = -1;

}

class Berth{
public:
    int id;
    int x;
    int y;
    int waitingBoatNum;
    int boatsIn[5]={-1,-1,-1,-1,-1}; //无船则为-1
    int transport_time;
    int loading_speed;
    int cargoNum;
    Berth():cargoNum(0){};
    void receive(int boatId);

    void lose(int boatId);

    void load();

}berths[berth_num + 10];

class Boat{
public:
    int capacity;
    int cargoNum;
    int id;
    int targetBerth;
    int state;
    int shippingTime;
    Boat():cargoNum(0),targetBerth(-1),state(1){};

    void moveAFrame();

    void action();

    void ship();

    void go();

    double value(int,int,int,int);
}boats[10];
void Boat::moveAFrame() {
    if(shippingTime>0){
        shippingTime--;
        if(shippingTime==0){
            if(targetBerth>=0){ //到达泊位
                berths[targetBerth].receive(Boat::id);
            }
            else{ //到达vp
                //ship();
            }
        }
    }
}

void Boat::action() {
    if (state == 1) { //available
        if (targetBerth == -1) { //at a vp
            ship();
            //state=0;
        }
        else { //at a berth
            Berth* berth=&berths[targetBerth];
            for(int i=0;i<berth->waitingBoatNum;i++){
                if(berth->boatsIn[i]==Boat::id){
                    if (cargoNum == capacity){
                        go();
                        berths[targetBerth].lose(Boat::id);
                        //state=0;
                    }
                    return;
                }
            }
            berth->receive(Boat::id);
        }
    }
}

void Boat::ship() {
    double v=0;
    int targetBerth=-1;
    int transportTime;
    int loadingSpeed;
    int cargoNum;
    int waitingBoatNum;

    for(int i=0;i<10;i++){
        transportTime=berths[i].transport_time;
        loadingSpeed=berths[i].loading_speed;
        cargoNum=berths[i].cargoNum;
        waitingBoatNum=berths[i].waitingBoatNum;

        double v_=value(transportTime,loadingSpeed,cargoNum,waitingBoatNum);
        if(v_>v){
            v=v_;
            targetBerth=i;
            shippingTime=berths[targetBerth].transport_time;
        }
    }

    cout<<"ship "<<Boat::id<<" "<<targetBerth<<endl;

}

double Boat::value(int transportTime, int loadingSpeed, int cargoNum, int waitingBoatNum) {
    if(cargoNum>capacity){
        cargoNum=capacity;
    }
    return
            (loadingSpeed+cargoNum)/(transportTime*(waitingBoatNum+1));
}

void Boat::go() {
    cout<<"go "<<Boat::id<<endl;
}

void Berth::load() {
    if(waitingBoatNum>0){
        Boat* boat=&boats[boatsIn[0]];
        int canLoadNum=cargoNum>=loading_speed ? loading_speed : cargoNum;
        if(boat->capacity-boat->cargoNum>=canLoadNum){
            boat->cargoNum+=canLoadNum;
            cargoNum-=canLoadNum;
        }
        else{
            cargoNum-=(boat->capacity-boat->cargoNum);
            boat->cargoNum=boat->capacity;
        }
    }
}

void Berth::receive(int boatId) {
    waitingBoatNum++;
    boatsIn[waitingBoatNum-1]=boatId;

//    if(waitingBoatNum>1){
//        boats[boatId].state=2;
//    }
}

void Berth::lose(int boatId) {
    //boats[boatsIn[0]].state=0;
    waitingBoatNum--;
    for(int i=1;i<=waitingBoatNum;i++){
        boatsIn[i-1]=boatsIn[i];
    }
    boatsIn[waitingBoatNum]=-1;
    //boats[boatsIn[0]].state=1;
}


class Gird{
public:
    char type;
    int robotId;
    Gird():robotId(-1){}; //-1表示无机器人

}Map[n][n];

int money, boat_capacity, id, cargoremained;//!

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
        Cargo* car = new Cargo;
        car->x=x;
        car->y=y;
        car->value=v;
        car->time=1000;
        cargos[x][y]=car;
    }

    //接下来10行robot数据
    for(int i = 0; i < robot_num; i ++)
    {
        //是否携带物品，坐标，状态（恢复状态还是正常状态）
        scanf("%d%d%d%d", &robots[i].carryState, &robots[i].x, &robots[i].y, &robots[i].movingState);
        //把机器人的id标注到地图上
        Map[robots[i].x][robots[i].y].robotId=i;
    }

    //接下来5行boat数据
    for(int i = 0; i < 5; i ++) {
        //船的状态（运输中，正常状态，泊位外等待状态），目标泊位
        scanf("%d%d\n", &boats[i].state, &boats[i].targetBerth);
        boats[i].action();
    }

    for(int i = 0;i<10;i++){
        berths[i].load();
    }

    char okk[100];
    scanf("%s", okk);
    return id;
}

Cargo* cargos[N][N];
for (int i = 0; i < N; i++)
{
    for(int j = 0; j < N; j++){
        cargos[i][j] = nullptr;
    }
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
