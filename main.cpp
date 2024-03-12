#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

class Grid{
public:
    char type;
    int robotId;
    Grid():robotId(-1){}; //-1表示无机器人

}Map[n][n];

struct Point
{
    int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F, G, H; //F=G+H
    Point *parent; //parent的坐标，这里没有用指针，从而简化代码
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化
    {
    }
};

bool isCanReach(Point *point)
{//下一步有机器人或者有障碍物和海洋
    if(Map[point->x][point->y].type=='#'||Map[point->x][point->y].type=='*'||Map[point->x][point->y].robotId!=-1)
        return false;
    return true;
}

class Astar
{
public:
    std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    std::vector<Point *> getSurroundPoints(const Point *point) const;
    Point *isInList(const std::list<Point *> &list, const Point *point) const; //判断开启/关闭列表中是否包含某点
    Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
    //计算FGH值
    int calcG(Point *temp_start, Point *point);
    int calcH(Point *point, Point *end);
    int calcF(Point *point);
private:
    std::list<Point *> openList;  //开启列表
    std::list<Point *> closeList; //关闭列表
};

int Astar::calcG(Point *temp_start, Point *point)
{
    int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空
    return parentG + 1;
}

int Astar::calcH(Point *point, Point *end)
{
    //用曼哈顿距离来当启发函数
    return ((end->x - point->x) + (end->y - point->y));
}

int Astar::calcF(Point *point)
{
    return point->G+point->H;
}

Point *Astar::getLeastFpoint()
{
    if (!openList.empty())
    {
        auto resPoint = openList.front();
        for (auto &point : openList)
            if (point->F<resPoint->F)
                resPoint = point;
        return resPoint;
    }
    return NULL;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
    openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
    while (!openList.empty())
    {
        auto curPoint = getLeastFpoint(); //找到F值最小的点
        openList.remove(curPoint); //从开启列表中删除
        closeList.push_back(curPoint); //放到关闭列表
        //1,找到当前周围八个格中可以通过的格子
        auto surroundPoints = getSurroundPoints(curPoint);
        for (auto &target : surroundPoints)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            if (!isInList(openList, target))
            {
                target->parent = curPoint;

                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->F = calcF(target);

                openList.push_back(target);
            }
                //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
            else
            {
                int tempG = calcG(curPoint, target);
                if (tempG<target->G)
                {
                    target->parent = curPoint;

                    target->G = tempG;
                    target->F = calcF(target);
                }
            }
            Point *resPoint = isInList(openList, &endPoint);
            if (resPoint)
                return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
        }
    }

    return NULL;
}

std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
    Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
    std::list<Point *> path;
    //返回路径，如果没找到路径，返回空链表
    while (result)
    {
        path.push_front(result);
        result = result->parent;
    }

    // 清空临时开闭列表，防止重复执行GetPath导致结果异常
    openList.clear();
    closeList.clear();

    return path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
    //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
    for (auto p : list)
        if (p->x == point->x&&p->y == point->y)
            return p;
    return NULL;
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point) const
{//把上下左右四个点中能走的点加入到附近的点集合当中返回
    auto* up=new Point(point->x-1,point->y);
    auto* down=new Point(point->x+1,point->y);
    auto* left=new Point(point->x,point->y-1);
    auto* right=new Point(point->x,point->y+1);
    std::vector<Point *> surroundPoints;
    if(isCanReach(up))
        surroundPoints.push_back(up);
    if(isCanReach(down))
        surroundPoints.push_back(down);
    if(isCanReach(left))
        surroundPoints.push_back(left);
    if(isCanReach(right))
        surroundPoints.push_back(right);
    return surroundPoints;
}

class Cargo{
public:
    int value;
    int x, y; //xb,yb为目标泊位的坐标
    int time;
    int matched;
    int berthid;
    Cargo():time(1000),berthid(-1),matched(0){}; //初始化泊位坐标为（-1，-1, 可存在时间为-1

    int findBerth();
};
Cargo* cargos[N][N];

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
    int cargoVal;
    Berth():id(0),x(0),y(0),waitingBoatNum(0),transport_time(0),loading_speed(0),cargoNum(0),cargoVal(0){};
    void receive(int boatId);

    void lose(int boatId);

    void load();

}berths[berth_num + 10];

int Cargo::findBerth(){//evaluation function for each berth
    //parameter for each evaluating dimension
    int a = 1;//pace to berth
    int b = 1;//loading time
    int c = 1;//berth time
    int d = 1;//todo:shipnum

    int result = 0;
    Point start(x, y);
    Astar astar;
    Point end(0,0);
    std::list<Point *> path = astar.GetPath(start, end, false);

    int steps = path.size();//pace to berth by robot plan
    int max = 0;
    int targetid = 0;

    for (size_t i = 0; i < berth_num; i++)
    {
        int loadtime = berths[i].cargoNum/berths[i].loading_speed;//todo:cargonum after steps
//todo: whether there is a ship
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
    pair<int,int> targetCargo;
    int movingState;

    //一些初始化
    Robot():id(-1),cargoValue(0),carryState(0),targetBerth(-1),targetCargo(-1,-1),movingState(1) {}

    void planToGetOrPull();

    void planToMove();

    void get();

    void pull(int id);

    void findCargo();

}robots[robot_num + 10];

void Robot::get(){
    if(this->x==this->targetCargo.first&&this->y==this->targetCargo.second){

        Map[this->targetCargo.first][this->targetCargo.second].type = '.';
        this->cargoValue = cargos[this->targetCargo.first][this->targetCargo.second]->value;
        this->carryState = 1;
        this->targetBerth = cargos[this->targetCargo.first][this->targetCargo.second]->berthid;
        //if()check nullptr
        delete cargos[this->targetCargo.first][this->targetCargo.second];

    }else{
        //error
    }

}

void Robot::pull(int id){
    berths[id].cargoNum++;
    berths[id].cargoVal+=cargoValue;

    this->cargoValue = 0;
    this->carryState = 0;
    this->targetBerth = -1;

    targetCargo={-1,-1};
}

Point findNearestBerthGrid(const Point& robot, const Point& berth) {
    // 如果机器人在泊位内部，直接返回机器人位置
    if (robot.x >= berth.x && robot.x <= berth.x + 3 &&
        robot.y >= berth.y && robot.y <= berth.y + 3) {
        return robot;
    }

    // 计算机器人到泊位四个边缘上所有格子的距离，并找到最小距离
    int minDistance = INT_MAX;
    Point nearestGrid(0,0);

    // 计算机器人到泊位四个边缘上的格子的距离
    for (int i = berth.x; i <= berth.x + 3; ++i) {
        int distance = abs(robot.x - i) + abs(robot.y - berth.y); // 上边缘格子
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y;
        }

        distance = abs(robot.x - i) + abs(robot.y - (berth.y + 3)); // 下边缘格子
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y+3;
        }
    }

    for (int j = berth.y + 1; j < berth.y + 3; ++j) {
        int distance = abs(robot.x - berth.x) + abs(robot.y - j); // 左边缘格子
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = berth.x;
            nearestGrid.y = j;
        }

        distance = abs(robot.x - (berth.x + 3)) + abs(robot.y - j); // 右边缘格子
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = berth.x+3;
            nearestGrid.y = j;
        }
    }

    return nearestGrid;
}

void Robot::findCargo(){
    //if robot has no cargo, calculate the targetCargo.Vice versa, just go to the targetBerth.
    if(carryState==0){
        //结合货物价格、货物到泊位的距离和自身到货物的距离来综合选择
        int a=1;//货物value权重
        int b=1;//货物距离权重
        int c=1;//货物到泊位的距离权重
        double maxCargoVal=INT_MIN;
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                if(cargos[i][j]!=nullptr&&cargos[i][j]->matched==0){
                    Point p= findNearestBerthGrid(Point(x,y),Point(berths[targetBerth].x,berths[targetBerth].y));
                    int cargoToBerth = abs(p.x-cargos[i][j]->x) + abs(p.y-cargos[i][j]->y);
                    int robotToCargo = abs(x-cargos[i][j]->x) + abs(y-cargos[i][j]->y);

                    double val =(double)(a * cargos[i][j]->value) / (double)(c * cargoToBerth + b * robotToCargo);

                    if(val > maxCargoVal){
                        maxCargoVal = val;
                        targetCargo.first = cargos[i][j]->x;
                        targetCargo.second = cargos[i][j]->y;
                    }
                }
            }
        }
        cargos[targetCargo.first][targetCargo.second]->matched=1;
    }

}

void Robot::planToGetOrPull() {
    //before move
    //detect can we get or pull?
    if(carryState==0&&x==targetCargo.first&&x==targetCargo.second){//手上没东西，且走到了目标货物的位置
        get();
        cout << "get " << id;
        return;
    }
    if(carryState==1&&berths[targetBerth].x<=x&&x<=berths[targetBerth].x+3&&berths[targetBerth].y<=y&&y<=berths[targetBerth].y+3){
        //手上有东西而且走到了泊位的范围
        pull(targetBerth);
        cout << "pull " << id;
    }
}

void Robot::planToMove(){
    //move
    //确定是往上下左右哪个方向移动
    Point end(0,0);
    if(carryState==1){
        end = findNearestBerthGrid(Point(x,y),Point(berths[targetBerth].x,berths[targetBerth].y));
    }
    else{
        end.x=targetCargo.first;
        end.y=targetCargo.second;
    }
    auto* up=new Point(x-1,y);
    auto* down=new Point(x+1,y);
    auto* left=new Point(x,y-1);
    auto* right=new Point(x,y+1);
    if(end.x>=x&&end.y>y){
        //目标在自己的右下方或者右方，先看右边（x，y+1），再看下边(x+1,y)的格子
        if(isCanReach(right)){//向右
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(down)){//向下
            cout << "move " << id << " " << 3;
            Map[x][y].robotId=-1;
            x++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(up)){//向上
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(left)) {//向左
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
    }
    else if(end.y<=y&&end.x>x){
        //目标在自己的左下方或者下方，先看下边（x+1，y），再看左边(x,y-1)的格子
        if(isCanReach(down)){//向下
            cout << "move " << id << " " << 3;
            Map[x][y].robotId=-1;
            x++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(left)) {//向左
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(right)){//向右
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(up)){//向上
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
    }
    else if(end.y<y&&end.x<=x){
        //目标在自己的左上方或者左方，先看左边（x+1，y），再看上边(x,y-1)的格子
        if(isCanReach(left)) {//向左
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(up)){//向上
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(down)){//向下
            cout << "move " << id << " " << 3;
            Map[x][y].robotId=-1;
            x++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(right)){//向右
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
    }
    else if(end.y<y&&end.x<=x){
        //目标在自己的右上方或者上方，先看上边（x+1，y），再看右边(x,y-1)的格子
        if(isCanReach(up)){//向上
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(right)){//向右
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(left)) {//向左
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(down)){//向下
            cout << "move " << id << " " << 3;
            Map[x][y].robotId=-1;
            x++;
            Map[x][y].robotId=id;
        }
    }
//    Point start(x, y);
//    Astar astar;
//    Point end(0,0);
//    if(carryState==1){
//        end.x=berths[targetBerth].x;
//        end.y=berths[targetBerth].y;
//    }
//    else{
//        end.x=targetCargo.first;
//        end.y=targetCargo.second;
//    }
//    std::list<Point *> path = astar.GetPath(start, end, false);
//    std::list<Point*>::iterator it = path.begin();
//    std::advance(it, 1); // 将迭代器向前移动一个位置，即跳过第一个元素
//    Point* next = *it;
//    if(next->x-x==1){//向下
//        cout << "move " << id << " " << 3;
//        Map[x][y].robotId=-1;
//        x++;
//        Map[x][y].robotId=id;
//    }
//    else if(next->x-x==-1){
//        cout << "move " << id << " " << 2;
//        Map[x][y].robotId=-1;
//        x--;
//        Map[x][y].robotId=id;
//    }
//    else if(next->y-y==-1){
//        cout << "move " << id << " " << 1;
//        Map[x][y].robotId=-1;
//        y--;
//        Map[x][y].robotId=id;
//    }
//    else if(next->y-y==1){
//        cout << "move " << id << " " << 0;
//        Map[x][y].robotId=-1;
//        y++;
//        Map[x][y].robotId=id;
//    }
}

class Boat{
public:
    int capacity;
    int cargoNum;
    int id;
    int targetBerth;
    int state;
    int shippingTime;
    Boat():cargoNum(0),targetBerth(-1),state(1){};

    //void moveAFrame();

    void action();

    void ship();

    void go();

    double value(int,int,int,int);
}boats[10];
//void Boat::moveAFrame() {
//    if(shippingTime>0){
//        shippingTime--;
//        if(shippingTime==0){
//            if(targetBerth>=0){ //到达泊位
//                berths[targetBerth].receive(Boat::id);
//            }
//            else{ //到达vp
//                //ship();
//            }
//        }
//    }
//}

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

int money, boat_capacity, id;

void Init()
{
    for (auto & cargo : cargos)
    {
        for(auto & j : cargo){
            j = nullptr;
        }
    }
    char line[N];
    for(auto & i : Map){
        scanf("%s", line); //地图输入
        //std::printf(line);
        for(int j=0;j<n;j++){
            switch (line[j]) {
                case 'B':{
                    i[j].type='B';
                    break;
                }
                case 'A':{
                    i[j].type='A';
                    break;
                }
                case '.':{
                    i[j].type='.';
                    break;
                }
                case '*':{
                    i[j].type='*';
                    break;
                }
                case '#':{
                    i[j].type='#';
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
        cargos[x][y]->x=x;
        cargos[x][y]->y=y;
        cargos[x][y]->value=v;
        cargos[x][y]->time=1000;
        //targetBerth的确定
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
    for(int i = 0; i < 5; i ++) {
        //船的状态（运输中，正常状态，泊位外等待状态），目标泊位
        scanf("%d%d\n", &boats[i].state, &boats[i].targetBerth);
    }
    char okk[100];
    scanf("%s", okk);
    return id;
}

bool compareForRobot(int id1,int id2) {
    //给机器人排序，根据排序后的序号来动机器人
    //首先，没货物的优先，因为货物可能会消失；其次没货物的当中谁的货物价值最大谁优先；有货物的也是谁的货物价值最大谁优先
    if(robots[id1].carryState==0&&robots[id2].carryState==1)
        return false;
    if(robots[id2].carryState==0&&robots[id1].carryState==1)
        return true;
    if(robots[id1].carryState==1&&robots[id2].carryState==1)
        return robots[id1].cargoValue < robots[id2].cargoValue;
    if(robots[id1].carryState==0&&robots[id2].carryState==0)
        return cargos[robots[id1].x][robots[id1].y]->value < cargos[robots[id2].x][robots[id2].y]->value;
}

int main()
{
    Init();
    for(int zhen = 1; zhen <= 15000; zhen ++)
    {
        int id = Input();
        std::vector<int> vec = {0,1,2,3,4,5,6,7,8,9};
        std::sort(vec.begin(), vec.end(), compareForRobot);
        for(auto robot:vec)
            if(robots[robot].carryState==0)
            robots[robot].findCargo();
        for(auto robot:vec)
            robots[robot].planToGetOrPull();
        for(auto robot:vec)
            robots[robot].planToMove();
        for(auto robot:vec)
            robots[robot].planToGetOrPull();
        //船舶指令


        for(int i=0;i<5;i++){
            boats[i].action();
        }
        for(int i = 0;i<10;i++){
            berths[i].load();
        }

        //泊位装卸货物
        puts("OK");
        fflush(stdout);
    }

    return 0;
}