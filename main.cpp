#include <bits/stdc++.h>
//#include <iostream>
//#include <vector>
//#include <algorithm>
//#include <list>
using namespace std;

int obstacleNum=0;
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;
int cargoSum = 0;

class Grid{
public:
    char type;
    int robotId;
    Grid():robotId(-1){}; //-1表示无机器人

}Map[n][n];

class Point
{
public:
    int x, y;
    int F, G, H; //F=G+H
    Point():x(-1),y(-1){};
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0){}
    Point(const Point& other) : x(other.x), y(other.y), F(other.F),G(other.G),H(other.H) {} // 拷贝构造函数

    //提供给map使用
    bool operator<(const Point& other) const {
        if (x != other.x) {
            return x < other.x;
        }
        return y < other.y;
    }

    bool operator == (const Point &p) const{
        return this->x==p.x && this->y==p.y;
    }

};

struct cmp
{
    bool operator()(const Point& p1, const Point& p2)
    {
        return p1.F > p2.F;
    }
};

class Berth{
public:
    int id;
    int x;
    int y;
    bool matched;
    int BoatNum;
    int boatsIn[5]={-1,-1,-1,-1,-1}; //无船则为-1
    int transport_time;
    int loading_speed;
    int cargoNum;
    int cargoVal;
    queue<int> cargoValues;
    Berth():id(0),x(0),y(0),BoatNum(0),transport_time(0),loading_speed(0),cargoNum(0),cargoVal(0){};
    void receive(int boatId);

    void lose(int boatId);

    void load();

    void expect(int);

}berths[berth_num + 10];

Point findNearestBerthGrid(const Point& robot, const Point& berth) {
    // 如果机器人在泊位内部，直接返回机器人位置
    if (robot.x >= berth.x && robot.x <= berth.x + 3 &&
        robot.y >= berth.y && robot.y <= berth.y + 3) {
        return robot;
    }

    // 计算机器人到泊位四个边缘上所有格子的距离，并找到最小距??
    int minDistance = INT_MAX;
    Point nearestGrid(0,0);

    // 计算机器人到泊位四个边缘上的格子的距??
    for (int i = berth.x; i <= berth.x + 3; ++i) {
        int distance = abs(robot.x - i) + abs(robot.y - berth.y); // 上边缘格??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y;
        }

        distance = abs(robot.x - i) + abs(robot.y - (berth.y + 3)); // 下边缘格??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y+3;
        }
    }

    for (int j = berth.y + 1; j < berth.y + 3; ++j) {
        int distance = abs(robot.x - berth.x) + abs(robot.y - j); // 左边缘格??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = berth.x;
            nearestGrid.y = j;
        }

        distance = abs(robot.x - (berth.x + 3)) + abs(robot.y - j); // 右边缘格??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = berth.x+3;
            nearestGrid.y = j;
        }
    }

    return nearestGrid;
}

bool isReachable(int x,int y)
{//下一步有障碍物和海洋
    if(y<0||y>=200||x<0||x>=200)//out of bound
        return false;
    if(Map[x][y].type=='#'||Map[x][y].type=='*')
        return false;
    return true;
}

bool isReachableWhenMoving(int x,int y){
    if(y<0||y>=200||x<0||x>=200)//out of bound
        return false;
    if(Map[x][y].robotId!=-1){
        return false;
    }
    return true;
}

int heuristic(int gx,int gy,int x,int y){
    return abs(gx-x)+abs(gy-y);
}

void findNeighbors(vector<Point>& neighbors,int x,int y){
    neighbors.clear();
    if(isReachable(x,y+1)){
        neighbors.push_back(Point(x,y+1));
    }
    if(isReachable(x,y-1)){
        neighbors.push_back(Point(x,y-1));
    }
    if(isReachable(x-1,y)){
        neighbors.push_back(Point(x-1,y));
    }
    if(isReachable(x+1,y)){
        neighbors.push_back(Point(x+1,y));
    }
}

class Cargo{
public:
    int value;
    int x, y;
    int time;
    int matched;
    int berthId;
    int stepsToBerth;
    stack<Point> path;
    queue<Point> temp;

    Cargo():time(1000),berthId(-1),matched(0){}; //初始化泊位坐标为??-1??-1, 可存在时间为-1

    Cargo(int _x,int _y,int v):x(_x),y(_y),value(v),time(1000),berthId(-1),matched(0){//targetBerth的确定改为机器人get到它再确定，节省时间
    }

    void findBerth();

    int A_FindBerth();

};Cargo* cargos[N][N];

void Cargo::findBerth(){
    int a = 2;//pace to berth
    int b = 1;//loading time
    int c = 1;//berth time
    int d = 1;//todo:shipnum
    int min=INT_MAX;
    int result=0;
    int steps=0;
    for(int i=0;i<10;i++){
        Berth br=berths[i];
        steps= heuristic(br.x,br.y,x,y);
        int loadTime = br.cargoNum/br.loading_speed;//todo:cargonum after steps
        //todo: whether there is a ship
        if(loadTime <= steps){
            result = a*steps+c*br.transport_time;
        }else{
            result = a*steps+b*(loadTime-steps)+c*br.transport_time;
        }
        if(min > result){
            min = result;
            berthId = i;
            stepsToBerth=steps;
        }
    }
}

int Cargo::A_FindBerth() {
    //parameter for each evaluating dimension
    int a = 1;//pace to berth
    int b = 1;//loading time
    int c = 1;//berth time
    int d = 1;//todo:shipnum

    int result = 0;
    Point start(x, y);
    int steps;
    int min = INT_MAX;
    int targetid = 0;
    map<Point,Point> came_from;

    for (size_t i = 0; i < berth_num; i++)
    {
        //Point targetPos=findNearestBerthGrid(Point(x,y),Point(berths[i].x,berths[i].y));
        Point targetPos(berths[i].x,berths[i].y);
        //终点
        Point target;
        //现在开始计算路径长度
        priority_queue<Point,vector<Point>,cmp> frontier;
        Point start(this->x,this->y);
        frontier.push(start);
        map<Point,int> cost_so_far;
        cost_so_far[start]=0;

        vector<Point> neighbors;

        while(frontier.size()!=0){
            Point current = frontier.top();
            frontier.pop();

            if(current.x==targetPos.x && current.y==targetPos.y){
                target=current;
                break;
            }

            findNeighbors(neighbors,current.x,current.y);

            for(Point next:neighbors){
                int new_cost = cost_so_far[current]+1;
                if(cost_so_far.find(next) == cost_so_far.end() || new_cost<cost_so_far[next]){
                    cost_so_far[next]=new_cost;
                    next.G=new_cost;
                    next.H=heuristic(targetPos.x,targetPos.y,next.x,next.y);
                    next.F=next.G+next.H;
                    frontier.push(next);
                    came_from[next]=current;
                }
            }
        }

        if(target.x==-1){
            steps=INT_MAX;
        }
        else{
            Point backPos(target);
            while(!(backPos==start)){
                temp.push(backPos);
                backPos=came_from[backPos];
            }
            steps=(int)temp.size();
        }

        came_from.clear();

        //Cargo找到路径之后，机器人直接把他的copy过来就好了，就不用再计算一遍到berth的路径了

        int loadtime = berths[i].cargoNum/berths[i].loading_speed;//todo:cargonum after steps
        //todo: whether there is a ship
        if(loadtime <= steps){
            result = a*steps+c*berths[i].transport_time;
        }else{
            result = a*steps+b*(loadtime-steps)+c*berths[i].transport_time;
        }

        if(min > result){
            while(!path.empty()){
                path.pop();
            }
            min = result;
            targetid = i;
            stepsToBerth=steps;
            while(!temp.empty()){
                path.push(temp.front());
                temp.pop();
            }
        }
    }
    while(!temp.empty()){
        temp.pop();
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
    pair<int,int> targetPos;
    stack<Point> path;
    bool trapped;
    //map<Point,Point> came_from;
    int needToGiveAway;//需要让路
    int movingState;
    int stepsToBerth;
    int stepsToCargo;

    //一些初始化
    Robot():id(-1),cargoValue(0),carryState(0),targetBerth(-1),targetPos(-1,-1),needToGiveAway(-1),movingState(1),stepsToBerth(-1),stepsToCargo(-1),trapped(
            false) {}

    void findPath();

    void planToGetOrPull();

    void move();

    void realMove();

    void get();

    void pull(int id);

    void findCargo();

    void giveAway(int dir);
}robots[robot_num + 10];

class Boat{
public:
    int capacity;
    int cargoNum;
    int id;
    int targetBerth;
    int state;
    int shippingTime;
    Boat():cargoNum(0),targetBerth(-1),state(1){};

    void action();

    void ship();

    void go();

    double value(int);
}boats[10];

void Robot::findPath() {
    Point target;
    map<Point,Point> came_from;

    priority_queue<Point,vector<Point>,cmp> frontier;
    Point start(this->x,this->y);
    frontier.push(start);

    map<Point,int> cost_so_far;
    cost_so_far[start]=0;

    vector<Point> neighbors;

    while(frontier.size()!=0){
        Point current = frontier.top();
        frontier.pop();

        if(current.x==targetPos.first && current.y==targetPos.second){
            target=current;
            break;
        }

        findNeighbors(neighbors,current.x,current.y);

        if(!neighbors.empty()){
            for(Point next:neighbors){
                int new_cost = cost_so_far[current]+1;
                if(cost_so_far.find(next) == cost_so_far.end() || new_cost<cost_so_far[next]){
                    cost_so_far[next]=new_cost;
                    next.G=new_cost;
                    next.H=heuristic(targetPos.first,targetPos.second,next.x,next.y);
                    next.F=next.G+next.H;
                    frontier.push(next);
                    came_from[next]=current;
                }
            }
        }
    }

    if(target.x==-1){
        if(targetBerth==-1){ //target cargo is trapped
            targetPos={-1,-1};
        }
        return;
    }

    Point backPos(target);
    while(!(backPos==start)){
        path.push(backPos);
        backPos=came_from[backPos];
    }

    if(carryState==0 && targetBerth==-1){
        stepsToCargo=path.size();
    }

    else if(targetBerth!=-1 && carryState==1){
        stepsToBerth=path.size();
    }
}

void Robot::get(){
    cargos[x][y]->findBerth();
        targetBerth=cargos[x][y]->berthId;
        //Point targetPoint(findNearestBerthGrid(Point(x,y),Point(berths[targetBerth].x,berths[targetBerth].y)));
        targetPos.first=berths[targetBerth].x;
        targetPos.second=berths[targetBerth].y;

        while (!path.empty()) {
            path.pop();
        }

        findPath();

//        stack<Point> tempPath;
//        while(!cargos[x][y]->path.empty()){
//            tempPath.push(cargos[x][y]->path.top());
//            cargos[x][y]->path.pop();
//        }
//        while(!tempPath.empty()){
//            path.push(tempPath.top());
//            tempPath.pop();
//        }

        cargoValue = cargos[x][y]->value;
        carryState = 1;
        stepsToCargo=-1;
        stepsToBerth=(int)path.size();

        Map[x][y].type = '.';

        delete cargos[x][y];
        cargos[x][y]= nullptr;
        cargoSum--;
}

void Robot::pull(int id){
    cargoValue = 0;
    carryState = 0;
    targetBerth = -1;

    targetPos={-1,-1};
    stepsToCargo=-1;
    stepsToBerth=-1;

    while (!path.empty()) {
        path.pop();
    }

    berths[id].cargoNum++;
    berths[id].cargoVal+=cargoValue;
    berths[id].cargoValues.push(cargoValue);
}

void Robot::findCargo(){
    //if robot has no cargo, calculate the targetPos.Vice versa, just go to the targetBerth.

    //结合货物价格、货物到泊位的距离和自身到货物的距离来综合选择
    int a=1;//货物value权重
    int b=1;//货物距离权重
    int c=1;//货物到泊位的距离权重
    double maxCargoVal=INT_MIN;
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            if(cargos[i][j]!=nullptr && cargos[i][j]->matched==0){
                //Point p= findNearestBerthGrid(Point(x,y),Point(berths[targetBerth].x,berths[targetBerth].y));

                int cargoToBerth = cargos[i][j]->stepsToBerth;
                int robotToCargo = abs(x-cargos[i][j]->x) + abs(y-cargos[i][j]->y);

                double val =(double)(a * cargos[i][j]->value) / (double)(c * cargoToBerth + b * robotToCargo);

                if(val > maxCargoVal){
                    maxCargoVal = val;
                    targetPos.first = cargos[i][j]->x;
                    targetPos.second = cargos[i][j]->y;
                }
            }
        }
    }
    //如果无可选的货物，否则不处理，防止数组越界
    if(targetBerth==-1&&targetPos.first>=0&&targetPos.second>=0){
        cargos[targetPos.first][targetPos.second]->matched=1;
    }

}

void Robot::planToGetOrPull() {
    //before move
    //detect can we get or pull?
    if(carryState==0 && targetBerth==-1 && x==targetPos.first && y==targetPos.second && cargos[x][y] != nullptr){//手上没东西，且走到了目标货物的位??
        get();
        printf("get %d\n",id);
        return;
    }
    if(carryState==1 && targetBerth>=0 && berths[targetBerth].x<=x&&x<=berths[targetBerth].x+3&&berths[targetBerth].y<=y&&y<=berths[targetBerth].y+3){
        //手上有东西而且走到了泊位的范围
        pull(targetBerth);
        printf("pull %d\n",id);
    }
}

void Robot::realMove() {
    if(!path.empty()){
        Point next=path.top();
        int x_=next.x,y_=next.y,dir;
        if(x==x_ && y<y_){
            dir=0;
        }
        else if(x==x_ && y>y_){
            dir=1;
        }
        else if(y==y_ && x>x_){
            dir=2;
        }
        else if(y==y_ && x<x_){
            dir=3;
        }//就算下一步有机器人也需要dir来转方向
        if(needToGiveAway==1){
            int nextDir;
            if(dir==0||dir==1){
                //如果本来是往水平方向行走
                if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){
                    nextDir=2;
                }
                else if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){
                    nextDir=3;
                }
                else{//前两个方向都走不了
                    if(dir==0){//如果本来向右走，那就只能向左走了
                        if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//左边可以走
                            nextDir=1;
                        }
                        else{//再让不了路，那就都别走了
                        }
                    }
                    if(dir==1){//如果本来向左走，那就只能向右走了
                        if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//右边可以走
                            nextDir=0;
                        }
                        else{//再让不了路，那就都别走了
                        }
                    }
                }
            }
            else if(dir==2||dir==3){//如果本来是往竖直方向行走
                if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//right
                    nextDir=0;
                }
                else if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//left
                    nextDir=1;
                }
                else{//前两个方向都走不了
                    if(dir==2){//如果本来向上走，那就只能向下走了
                        if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){//下边可以走
                            nextDir=3;
                        }
                        else{//再让不了路，那就都别走了
                        }
                    }
                    if(dir==1){//如果本来向下走，那就只能向上走了
                        if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){//右边可以走
                            nextDir=2;
                        }
                        else{//再让不了路，那就都别走了
                        }
                    }
                }
            }
            needToGiveAway=0;//下次就不用让路了
        }
        else{
            if(isReachableWhenMoving(x_,y_)){//发现下一步没有机器人
                path.pop();
                printf("move %d %d\n",id,dir);

                Map[x][y].robotId=-1;
                x=x_;
                y=y_;
                Map[x][y].robotId=id;

                if(targetBerth!=-1){ //going to berth
                    stepsToBerth--;
                }
                else{
                    stepsToCargo--;
                }
            }
            else{//发现下一步有机器人
                if(Map[next.x][next.y].robotId==-1)//为了确保不取负值
                    return;
                else {
                    Robot ro = robots[Map[next.x][next.y].robotId];
                    Point roNextStep = ro.path.top();
                    if(roNextStep.x==x&&roNextStep.y==y){//G！方向相反
                        int theSmallOne=0;//如果是0则为自身，如果是1则为对方
                        //二者进行比较，首先，没货物的优先，因为货物可能会消失；其次没货物的当中谁的货物价值最大谁优先；有货物的也是谁的货物价值最大谁优先
                        if(carryState==0&&ro.carryState==1)
                            theSmallOne = 1;
                        else if(carryState==1&&ro.carryState==1)
                            theSmallOne =  cargoValue < ro.cargoValue;
                        else if(carryState==0 && carryState==0){
                            if(targetPos.first!=-1 && ro.targetPos.first!=-1) {
                                theSmallOne = cargos[targetPos.first][targetPos.second]->value <
                                              cargos[ro.targetPos.first][ro.targetPos.second]->value;
                            }
                        }
                        //找到最小的是自己或者对方
                        if(theSmallOne==0){//是自己,那就自己让路,先看一下这个方向垂直的两个方向，最后再看反方向
                            giveAway(dir);
                        }
                        else{//是对方,那就通知对方让路
                            robots[Map[next.x][next.y].robotId].needToGiveAway=1;
                        }
                    }
                    else{//如果方向不是相反，让不影响对方的先走，因为自己的方向指向另一个机器人，所以应该是自己让路
                        giveAway(dir);
                    }
                }
            }
        }
    }
}

void Robot::giveAway(int dir){
    int nextDir;
    if(dir==0||dir==1){
        //如果本来是往水平方向行走
        if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){
            nextDir=2;
        }
        else if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){
            nextDir=3;
        }
        else{//前两个方向都走不了
            if(dir==0){//如果本来向右走，那就只能向左走了
                if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//左边可以走
                    nextDir=1;
                }
                else{//左边也走不了，说明自己被卡死了，只能让对方走
                    //做一些事情来标记对方，让他来执行这条让路指令（突然想到一个情况，如果一个机器人被另一个堵在角落，而且角落里的机器人还比较弱，他要怎么出来？）
                    robots[Map[x][y+1].robotId].needToGiveAway=1;
                }
            }
            if(dir==1){//如果本来向左走，那就只能向右走了
                if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//右边可以走
                    nextDir=0;
                }
                else{//右边也走不了，说明自己被卡死了，只能让对方走
                    robots[Map[x][y-1].robotId].needToGiveAway=1;
                }
            }
        }
    }
    else if(dir==2||dir==3){//如果本来是往竖直方向行走
        if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//right
            nextDir=0;
        }
        else if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//left
            nextDir=1;
        }
        else{//前两个方向都走不了
            if(dir==2){//如果本来向上走，那就只能向下走了
                if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){//下边可以走
                    nextDir=3;
                }
                else{//下边也走不了，说明自己被卡死了，只能让对方走
                    robots[Map[x-1][y].robotId].needToGiveAway=1;
                }
            }
            if(dir==1){//如果本来向下走，那就只能向上走了
                if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){//右边可以走
                    nextDir=2;
                }
                else{//上边也走不了，说明自己被卡死了，只能让对方走
                    robots[Map[x+1][y].robotId].needToGiveAway=1;
                }
            }
        }
    }
    printf("move %d %d\n",id,nextDir);
    path.push(Point(x,y));
    Map[x][y].robotId=-1;
    if(nextDir==0)
        y++;
    else if(nextDir==1)
        y--;
    else if(nextDir==2)
        x--;
    else x++;
    Map[x][y].robotId=id;

    if(targetBerth!=-1){ //going to berth
        stepsToBerth++;
    }
    else{
        stepsToCargo++;
    }
}

void Robot::move() {
    if(!path.empty()){
        realMove();
    }
    else{
        findPath();
        if(!path.empty()){
            realMove();
        }
    }
}

void Boat::action() {
    if (state == 1) { //available
        if (targetBerth == -1) { //at a vp
            ship();
        }
        else { //at a berth
            Berth* berth=&berths[targetBerth];
            for(int i=0;i<berth->BoatNum;i++){
                if(berth->boatsIn[i]==Boat::id){
                    if (cargoNum == capacity){
                        go();
                        berths[targetBerth].lose(Boat::id);
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
    for(int i=0;i<10;i++){
        double v_=value(i);
        if(v_>v){
            v=v_;
            targetBerth=i;
        }
    }
    if(targetBerth>=0){
        berths[targetBerth].matched=true;

        printf("ship %d %d\n",id,targetBerth);
    }


}

double Boat::value(int berthId) {
    Berth berth=berths[berthId];
    int canLoadCargoValue=0;
    int canLoadCargoNum;
    int time=1;
    if(berth.BoatNum>0){
        if(berth.matched){
            return -1;
        }

        Boat boat=boats[berth.boatsIn[0]];
        int loadTime= ::ceil((boat.capacity-boat.cargoNum)/berth.loading_speed);
        int waitingTime=loadTime-berth.transport_time>0 ? loadTime-berth.transport_time : 0;
        berth.expect(berth.transport_time);
        canLoadCargoNum=boat.capacity-waitingTime*berth.loading_speed;
        canLoadCargoNum=canLoadCargoNum > berth.cargoNum ? berth.cargoNum : canLoadCargoValue;
        for(int i=0;i<canLoadCargoNum;i++){
            canLoadCargoValue+=berth.cargoValues.front();
            berth.cargoValues.pop();
        }
        time=berth.transport_time+::ceil(canLoadCargoNum/berth.loading_speed)+waitingTime;
    }
    else{
        if(berth.matched){
            return -1;
        }

        berth.expect(berth.transport_time);
        canLoadCargoNum=capacity > berth.cargoNum ? berth.cargoNum : capacity;
        for(int i=0;i<canLoadCargoNum;i++){
            canLoadCargoValue+=berth.cargoValues.front();
            berth.cargoValues.pop();
        }
        time=berth.transport_time+::ceil(canLoadCargoNum/berth.loading_speed);
    }
    return canLoadCargoValue/time;
}

void Boat::go() {
    printf("go %d\n",id);
}

void Berth::load() {
    if(BoatNum>0){
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
    BoatNum++;
    boatsIn[BoatNum-1]=boatId;
    matched= false;
}

void Berth::lose(int boatId) {
    //boats[boatsIn[0]].state=0;
    BoatNum--;
    for(int i=1;i<=BoatNum;i++){
        boatsIn[i-1]=boatsIn[i];
    }
    boatsIn[BoatNum]=-1;
    //boats[boatsIn[0]].state=1;
}

void Berth::expect(int time) {
    for(int i=0;i<10;i++){
        Robot r=robots[i];
        if(r.stepsToBerth<=time){
            if(r.targetBerth==Berth::id){
                cargoValues.push(r.cargoValue);
                cargoNum++;
            }
        }
    }
}

bool compareForRobot(int id1,int id2) {
    Robot r1=robots[id1];
    Robot r2=robots[id2];
    //给机器人排序，根据排序后的序号来动机器人
    //首先，没货物的优先，因为货物可能会消失；其次没货物的当中谁的货物价值最大谁优先；有货物的也是谁的货物价值最大谁优先
    if(r1.trapped){
        return false;
    }
    if (r2.trapped){
        return true;
    }
    if(r1.carryState==0&&r2.carryState==1)
        return false;
    if(r2.carryState==0&&r1.carryState==1)
        return true;
    if(r1.carryState==1&&r2.carryState==1)
        return robots[id1].cargoValue < robots[id2].cargoValue;
    if(r1.carryState==0 && r2.carryState==0){
        if(r1.targetPos.first<0 && r2.targetPos.first<0) {
            return cargos[r1.targetPos.first][r1.targetPos.second]->value <
                   cargos[r2.targetPos.first][r2.targetPos.second]->value;
        }
        return true;
    }
    return true;
}

int money, boat_capacity, id;

void Init()
{
    for (int i=0;i<210;i++)
    {
        for(int j =0;j<210;j++){
            cargos[i][j] = nullptr;
        }
    }

    char line[N];
    for(auto & i : Map){
        scanf("%s", line); //地图输入
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
                    obstacleNum++;
                    break;
                }
                case '#':{
                    i[j].type='#';
                    obstacleNum++;
                    break;
                }

                default:break;

            }
        }
    }

    for(int i = 0; i < berth_num; i ++)
    {
        int id_,x,y,t,v;
        //港口的id,坐标，运输到虚拟点的时间，装载速度
        scanf("%d%d%d%d%d",&id_ ,&x, &y, &t, &v);
        berths[id_].id=id_;
        berths[id_].x=x;
        berths[id_].y=y;
        berths[id_].transport_time=t;
        berths[id_].loading_speed=v;
    }

    //初始化船的属??
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

bool isRobotTrapped(const Point& start) {
    // 定义方向数组，用于获取相邻位置
    const vector<pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };

    // 使用 BFS 搜索可达位置
    queue<Point> q;
    unordered_set<int> visited; // 使用哈希集合来记录已访问的位置

    q.push(start);
    visited.insert(start.x * 1000 + start.y); // 将起始位置转换为哈希值并加入集合

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        // 检查当前位置的相邻位置
        for (const auto& dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;

            // 如果相邻位置在地图范围内且未被访问且不是障碍物，则加入队列并标记为已访问
            if (newX >= 0 && newX < 200 && newY >= 0 && newY < 200 &&
                Map[newX][newY].type != '#' && Map[newX][newY].type != '*' &&
                visited.find(newX * 1000 + newY) == visited.end()) {
                q.push(Point(newX, newY));
                visited.insert(newX * 1000 + newY);
                if(visited.size()>=900){
                    return false;
                }
            }
        }
    }
    return true;
}

void Input(){
    scanf("%d%d", &id, &money); //帧序号，当前金钱??

    int num; //新增的货物量
    scanf("%d", &num);
    cargoSum+=num;
    for(int i = 0; i < num; i ++)
    {
        int x,y,v;
        scanf("%d%d%d", &x,&y,&v);  //货物的坐标和金额
        auto* car = new Cargo(x,y,v);
        cargos[x][y]=car;
    }

    //遍历货物，生存时间减一
    for (int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++){
            if(cargos[i][j]!=nullptr){
                cargos[i][j]->time--;
                if(cargos[i][j]->time==0){
                    delete cargos[i][j];
                    cargos[i][j]= nullptr;
                    cargoSum--;
                }
            }
        }
    }

    //接下??10行robot数据
    for(int i = 0; i < robot_num; i ++)
    {
        Robot* r=&robots[i];
        //是否携带物品，坐标，状态（恢复状态还是正常状态）
        scanf("%d%d%d%d", &r->carryState, &r->x, &r->y, &r->movingState);
        //把机器人的id标注到地图上
        Map[r->x][r->y].robotId=i;
    }

    //接下??5行boat数据
    for(int i = 0; i < 5; i ++) {
        //船的状态（运输中，正常状态，泊位外等待状态），目标泊??
        scanf("%d%d\n", &boats[i].state, &boats[i].targetBerth);
    }
    char okk[100];
    scanf("%s", okk);
}

int main() {
    Init();
    int robotCheckNum=0;

    for (int zhen = 1; zhen <= 15000; zhen++) {

        Input();

        if(zhen==1){
            for(int i=0;i<10;i++){
                Robot* r=&robots[i];
                r->trapped=isRobotTrapped(Point(r->x,r->y));
            }
        }

        //if(zhen <2000) {
            std::vector<int> vec = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

            //find cargo
            for (auto robot: vec) {
                Robot *r = &robots[robot];
                if (!r->trapped) {
                    if (r->carryState == 0) { //没拿着货物
                        if (r->targetPos.first == -1) { //没有目标货物
                            r->findCargo();
                        } else { //已经有目标货物
                            if (cargos[r->targetPos.first][r->targetPos.second] == nullptr) { //但是目标货物消亡
                                r->targetPos = {-1, -1};
                                r->findCargo();  //重新找
                            }
                        }
                    }
                }
            }

            std::sort(vec.begin(), vec.end(), compareForRobot);

            for (auto robot: vec) {
                Robot *r = &robots[robot];
                if (!r->trapped) {
                    if (r->movingState == 1) {
                        if (r->targetPos.first != -1) { //has target
                            r->move();
                        } else {
                            continue;
                        }
                    }
                }
            }

            for (auto robot: vec) {
                Robot *r = &robots[robot];
                if (!r->trapped) {
                    if (r->movingState == 1)
                        r->planToGetOrPull();
                }

            }

            //船舶指令
            if (zhen == 1) {
                for (int i = 0; i < 5; i++) {
                    printf("ship %d %d\n", i, i);
                }
            } else {
                for (int i = 0; i < 5; i++) {
                    boats[i].action();
                }
            }

            //泊位装卸货物
            for (int i = 0; i < 10; i++) {
                berths[i].load();
            }


            if (zhen == 3000) {
                for (int i = 0; i < 5; i++) {
                    boats[i].go();
                }
            }
        //}

        puts("OK");
        fflush(stdout);
    }
    return 0;
}
