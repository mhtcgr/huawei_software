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
    int boats[5]={-1,-1,-1,-1,-1}; //�޴���Ϊ-1
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
    Boat():cargoNum(0),targetBerth(-2),state(1){}; //Ŀ�겴λΪ-2��ʾ��ǰΪ��ʼ״̬����Ŀ��(��Ϊ-1��ʾ����㣩

    void ship();

    void go();
}boats[10];

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
    for(int i = 0; i < 5; i ++)
        //����״̬�������У�����״̬����λ��ȴ�״̬����Ŀ�겴λ
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
