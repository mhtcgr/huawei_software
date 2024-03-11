#include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

class Cargo{
public:
    int value;
    int x, y, xb, yb; //xb,ybΪĿ�겴λ������
    int time;
    Cargo():xb(-1),yb(-1),time(-1){}; //��ʼ����λ����Ϊ��-1��-1, �ɴ���ʱ��Ϊ-1

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

    //һЩ��ʼ��
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
    int waitingBoatNum;
    int boatsIn[5]={-1,-1,-1,-1,-1}; //�޴���Ϊ-1
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
            if(targetBerth>=0){ //���ﲴλ
                berths[targetBerth].receive(Boat::id);
            }
            else{ //����vp
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
    Gird():robotId(-1){}; //-1��ʾ�޻�����

}Map[n][n];

int money, boat_capacity, id;

void Init()
{
    char line[N];
    for(int i = 0; i < n; i ++){
        scanf("%s", line); //��ͼ����
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
        int id; //�ۿ�id
        scanf("%d", &id);
        berths[id].id=id;
        //�ۿڵ����꣬���䵽������ʱ�䣬װ���ٶ�
        scanf("%d%d%d%d", &berths[id].x, &berths[id].y, &berths[id].transport_time, &berths[id].loading_speed);
    }

    //��ʼ����������
    scanf("%d", &boat_capacity); //��������
    for(int i=0;i<5;i++){
        boats[i].capacity=boat_capacity;
        boats[i].id=i;
    }

    //��ʼ�������˵�id
    for(int i =0;i<10;i++){
        robots[i].id=i;
    }
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

int Input(){
    scanf("%d%d", &id, &money); //֡��ţ���ǰ��Ǯ��

    int num; //�����Ļ�����
    scanf("%d", &num);
    for(int i = 0; i < num; i ++)
    {
        int x,y,v;
        scanf("%d%d%d", &x,&y,&v);  //���������ͽ��
        cargos[x][y].x=x;
        cargos[x][y].y=y;
        cargos[x][y].value=v;
        cargos[x][y].time=1000;
    }

    //������Ҫ�����������ʱ���һ���Ҳ�λ

    //������10��robot����
    for(int i = 0; i < robot_num; i ++)
    {
        //�Ƿ�Я����Ʒ�����꣬״̬���ָ�״̬��������״̬��
        scanf("%d%d%d%d", &robots[i].carryState, &robots[i].x, &robots[i].y, &robots[i].movingState);
        //�ѻ����˵�id��ע����ͼ��
        Map[robots[i].x][robots[i].y].robotId=i;
    }

    //������5��boat����
    for(int i = 0; i < 5; i ++) {
        //����״̬�������У�����״̬����λ��ȴ�״̬����Ŀ�겴λ
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
