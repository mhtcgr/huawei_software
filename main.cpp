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
    Grid():robotId(-1){}; //-1��ʾ�޻�����

}Map[n][n];

class Point
{
public:
    int x, y;
    int F, G, H; //F=G+H
    Point():x(-1),y(-1){};
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0){}
    Point(const Point& other) : x(other.x), y(other.y), F(other.F),G(other.G),H(other.H) {} // �������캯��

    //�ṩ��mapʹ��
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
    int boatsIn[5]={-1,-1,-1,-1,-1}; //�޴���Ϊ-1
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
    // ����������ڲ�λ�ڲ���ֱ�ӷ��ػ�����λ��
    if (robot.x >= berth.x && robot.x <= berth.x + 3 &&
        robot.y >= berth.y && robot.y <= berth.y + 3) {
        return robot;
    }

    // ��������˵���λ�ĸ���Ե�����и��ӵľ��룬���ҵ���С��??
    int minDistance = INT_MAX;
    Point nearestGrid(0,0);

    // ��������˵���λ�ĸ���Ե�ϵĸ��ӵľ�??
    for (int i = berth.x; i <= berth.x + 3; ++i) {
        int distance = abs(robot.x - i) + abs(robot.y - berth.y); // �ϱ�Ե��??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y;
        }

        distance = abs(robot.x - i) + abs(robot.y - (berth.y + 3)); // �±�Ե��??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y+3;
        }
    }

    for (int j = berth.y + 1; j < berth.y + 3; ++j) {
        int distance = abs(robot.x - berth.x) + abs(robot.y - j); // ���Ե��??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = berth.x;
            nearestGrid.y = j;
        }

        distance = abs(robot.x - (berth.x + 3)) + abs(robot.y - j); // �ұ�Ե��??
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = berth.x+3;
            nearestGrid.y = j;
        }
    }

    return nearestGrid;
}

bool isReachable(int x,int y)
{//��һ�����ϰ���ͺ���
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

    Cargo():time(1000),berthId(-1),matched(0){}; //��ʼ����λ����Ϊ??-1??-1, �ɴ���ʱ��Ϊ-1

    Cargo(int _x,int _y,int v):x(_x),y(_y),value(v),time(1000),berthId(-1),matched(0){//targetBerth��ȷ����Ϊ������get������ȷ������ʡʱ��
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
        //�յ�
        Point target;
        //���ڿ�ʼ����·������
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

        //Cargo�ҵ�·��֮�󣬻�����ֱ�Ӱ�����copy�����ͺ��ˣ��Ͳ����ټ���һ�鵽berth��·����

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
    int needToGiveAway;//��Ҫ��·
    int movingState;
    int stepsToBerth;
    int stepsToCargo;

    //һЩ��ʼ��
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

    //��ϻ���۸񡢻��ﵽ��λ�ľ������������ľ������ۺ�ѡ��
    int a=1;//����valueȨ��
    int b=1;//�������Ȩ��
    int c=1;//���ﵽ��λ�ľ���Ȩ��
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
    //����޿�ѡ�Ļ�����򲻴�����ֹ����Խ��
    if(targetBerth==-1&&targetPos.first>=0&&targetPos.second>=0){
        cargos[targetPos.first][targetPos.second]->matched=1;
    }

}

void Robot::planToGetOrPull() {
    //before move
    //detect can we get or pull?
    if(carryState==0 && targetBerth==-1 && x==targetPos.first && y==targetPos.second && cargos[x][y] != nullptr){//����û���������ߵ���Ŀ������λ??
        get();
        printf("get %d\n",id);
        return;
    }
    if(carryState==1 && targetBerth>=0 && berths[targetBerth].x<=x&&x<=berths[targetBerth].x+3&&berths[targetBerth].y<=y&&y<=berths[targetBerth].y+3){
        //�����ж��������ߵ��˲�λ�ķ�Χ
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
        }//������һ���л�����Ҳ��Ҫdir��ת����
        if(needToGiveAway==1){
            int nextDir;
            if(dir==0||dir==1){
                //�����������ˮƽ��������
                if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){
                    nextDir=2;
                }
                else if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){
                    nextDir=3;
                }
                else{//ǰ���������߲���
                    if(dir==0){//������������ߣ��Ǿ�ֻ����������
                        if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//��߿�����
                            nextDir=1;
                        }
                        else{//���ò���·���ǾͶ�������
                        }
                    }
                    if(dir==1){//������������ߣ��Ǿ�ֻ����������
                        if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//�ұ߿�����
                            nextDir=0;
                        }
                        else{//���ò���·���ǾͶ�������
                        }
                    }
                }
            }
            else if(dir==2||dir==3){//�������������ֱ��������
                if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//right
                    nextDir=0;
                }
                else if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//left
                    nextDir=1;
                }
                else{//ǰ���������߲���
                    if(dir==2){//������������ߣ��Ǿ�ֻ����������
                        if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){//�±߿�����
                            nextDir=3;
                        }
                        else{//���ò���·���ǾͶ�������
                        }
                    }
                    if(dir==1){//������������ߣ��Ǿ�ֻ����������
                        if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){//�ұ߿�����
                            nextDir=2;
                        }
                        else{//���ò���·���ǾͶ�������
                        }
                    }
                }
            }
            needToGiveAway=0;//�´ξͲ�����·��
        }
        else{
            if(isReachableWhenMoving(x_,y_)){//������һ��û�л�����
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
            else{//������һ���л�����
                if(Map[next.x][next.y].robotId==-1)//Ϊ��ȷ����ȡ��ֵ
                    return;
                else {
                    Robot ro = robots[Map[next.x][next.y].robotId];
                    Point roNextStep = ro.path.top();
                    if(roNextStep.x==x&&roNextStep.y==y){//G�������෴
                        int theSmallOne=0;//�����0��Ϊ���������1��Ϊ�Է�
                        //���߽��бȽϣ����ȣ�û��������ȣ���Ϊ������ܻ���ʧ�����û����ĵ���˭�Ļ����ֵ���˭���ȣ��л����Ҳ��˭�Ļ����ֵ���˭����
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
                        //�ҵ���С�����Լ����߶Է�
                        if(theSmallOne==0){//���Լ�,�Ǿ��Լ���·,�ȿ�һ���������ֱ��������������ٿ�������
                            giveAway(dir);
                        }
                        else{//�ǶԷ�,�Ǿ�֪ͨ�Է���·
                            robots[Map[next.x][next.y].robotId].needToGiveAway=1;
                        }
                    }
                    else{//����������෴���ò�Ӱ��Է������ߣ���Ϊ�Լ��ķ���ָ����һ�������ˣ�����Ӧ�����Լ���·
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
        //�����������ˮƽ��������
        if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){
            nextDir=2;
        }
        else if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){
            nextDir=3;
        }
        else{//ǰ���������߲���
            if(dir==0){//������������ߣ��Ǿ�ֻ����������
                if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//��߿�����
                    nextDir=1;
                }
                else{//���Ҳ�߲��ˣ�˵���Լ��������ˣ�ֻ���öԷ���
                    //��һЩ��������ǶԷ���������ִ��������·ָ�ͻȻ�뵽һ����������һ�������˱���һ�����ڽ��䣬���ҽ�����Ļ����˻��Ƚ�������Ҫ��ô��������
                    robots[Map[x][y+1].robotId].needToGiveAway=1;
                }
            }
            if(dir==1){//������������ߣ��Ǿ�ֻ����������
                if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//�ұ߿�����
                    nextDir=0;
                }
                else{//�ұ�Ҳ�߲��ˣ�˵���Լ��������ˣ�ֻ���öԷ���
                    robots[Map[x][y-1].robotId].needToGiveAway=1;
                }
            }
        }
    }
    else if(dir==2||dir==3){//�������������ֱ��������
        if(isReachable(x,y+1)&& isReachableWhenMoving(x,y+1)){//right
            nextDir=0;
        }
        else if(isReachable(x,y-1)&& isReachableWhenMoving(x,y-1)){//left
            nextDir=1;
        }
        else{//ǰ���������߲���
            if(dir==2){//������������ߣ��Ǿ�ֻ����������
                if(isReachable(x+1,y)&& isReachableWhenMoving(x+1,y)){//�±߿�����
                    nextDir=3;
                }
                else{//�±�Ҳ�߲��ˣ�˵���Լ��������ˣ�ֻ���öԷ���
                    robots[Map[x-1][y].robotId].needToGiveAway=1;
                }
            }
            if(dir==1){//������������ߣ��Ǿ�ֻ����������
                if(isReachable(x-1,y)&& isReachableWhenMoving(x-1,y)){//�ұ߿�����
                    nextDir=2;
                }
                else{//�ϱ�Ҳ�߲��ˣ�˵���Լ��������ˣ�ֻ���öԷ���
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
    //�����������򣬸����������������������
    //���ȣ�û��������ȣ���Ϊ������ܻ���ʧ�����û����ĵ���˭�Ļ����ֵ���˭���ȣ��л����Ҳ��˭�Ļ����ֵ���˭����
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
        scanf("%s", line); //��ͼ����
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
        //�ۿڵ�id,���꣬���䵽������ʱ�䣬װ���ٶ�
        scanf("%d%d%d%d%d",&id_ ,&x, &y, &t, &v);
        berths[id_].id=id_;
        berths[id_].x=x;
        berths[id_].y=y;
        berths[id_].transport_time=t;
        berths[id_].loading_speed=v;
    }

    //��ʼ��������??
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

bool isRobotTrapped(const Point& start) {
    // ���巽�����飬���ڻ�ȡ����λ��
    const vector<pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };

    // ʹ�� BFS �����ɴ�λ��
    queue<Point> q;
    unordered_set<int> visited; // ʹ�ù�ϣ��������¼�ѷ��ʵ�λ��

    q.push(start);
    visited.insert(start.x * 1000 + start.y); // ����ʼλ��ת��Ϊ��ϣֵ�����뼯��

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        // ��鵱ǰλ�õ�����λ��
        for (const auto& dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;

            // �������λ���ڵ�ͼ��Χ����δ�������Ҳ����ϰ���������в����Ϊ�ѷ���
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
    scanf("%d%d", &id, &money); //֡��ţ���ǰ��Ǯ??

    int num; //�����Ļ�����
    scanf("%d", &num);
    cargoSum+=num;
    for(int i = 0; i < num; i ++)
    {
        int x,y,v;
        scanf("%d%d%d", &x,&y,&v);  //���������ͽ��
        auto* car = new Cargo(x,y,v);
        cargos[x][y]=car;
    }

    //�����������ʱ���һ
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

    //����??10��robot����
    for(int i = 0; i < robot_num; i ++)
    {
        Robot* r=&robots[i];
        //�Ƿ�Я����Ʒ�����꣬״̬���ָ�״̬��������״̬��
        scanf("%d%d%d%d", &r->carryState, &r->x, &r->y, &r->movingState);
        //�ѻ����˵�id��ע����ͼ��
        Map[r->x][r->y].robotId=i;
    }

    //����??5��boat����
    for(int i = 0; i < 5; i ++) {
        //����״̬�������У�����״̬����λ��ȴ�״̬����Ŀ�겴??
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
                    if (r->carryState == 0) { //û���Ż���
                        if (r->targetPos.first == -1) { //û��Ŀ�����
                            r->findCargo();
                        } else { //�Ѿ���Ŀ�����
                            if (cargos[r->targetPos.first][r->targetPos.second] == nullptr) { //����Ŀ���������
                                r->targetPos = {-1, -1};
                                r->findCargo();  //������
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

            //����ָ��
            if (zhen == 1) {
                for (int i = 0; i < 5; i++) {
                    printf("ship %d %d\n", i, i);
                }
            } else {
                for (int i = 0; i < 5; i++) {
                    boats[i].action();
                }
            }

            //��λװж����
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
