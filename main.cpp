#include <bits/stdc++.h>
//#include <iostream>
//#include <vector>
//#include <algorithm>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

class Grid{
public:
    char type;
    int robotId;
    Grid():robotId(-1){}; //-1��ʾ�޻�����

}Map[n][n];

struct Point
{
    int x, y; //�����꣬����Ϊ�˷��㰴��C++�����������㣬x������ţ�y��������
    int F, G, H; //F=G+H
    Point *parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴���
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //������ʼ��
    {
    }
};

bool isCanReach(Point *point)
{//��һ���л����˻������ϰ���ͺ���
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
    Point *isInList(const std::list<Point *> &list, const Point *point) const; //�жϿ���/�ر��б����Ƿ����ĳ��
    Point *getLeastFpoint(); //�ӿ����б��з���Fֵ��С�Ľڵ�
    //����FGHֵ
    int calcG(Point *temp_start, Point *point);
    int calcH(Point *point, Point *end);
    int calcF(Point *point);
private:
    std::list<Point *> openList;  //�����б�
    std::list<Point *> closeList; //�ر��б�
};

int Astar::calcG(Point *temp_start, Point *point)
{
    int parentG = point->parent == NULL ? 0 : point->parent->G; //����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿ�
    return parentG + 1;
}

int Astar::calcH(Point *point, Point *end)
{
    //�������پ���������������
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
    openList.push_back(new Point(startPoint.x, startPoint.y)); //�������,��������һ���ڵ㣬�������
    while (!openList.empty())
    {
        auto curPoint = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
        openList.remove(curPoint); //�ӿ����б���ɾ��
        closeList.push_back(curPoint); //�ŵ��ر��б�
        //1,�ҵ���ǰ��Χ�˸����п���ͨ���ĸ���
        auto surroundPoints = getSurroundPoints(curPoint);
        for (auto &target : surroundPoints)
        {
            //2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ��Ϊ�丸�ڵ㣬����F G H
            if (!isInList(openList, target))
            {
                target->parent = curPoint;

                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->F = calcF(target);

                openList.push_back(target);
            }
                //3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
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
                return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
        }
    }

    return NULL;
}

std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
    Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
    std::list<Point *> path;
    //����·�������û�ҵ�·�������ؿ�����
    while (result)
    {
        path.push_front(result);
        result = result->parent;
    }

    // �����ʱ�����б���ֹ�ظ�ִ��GetPath���½���쳣
    openList.clear();
    closeList.clear();

    return path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
    //�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
    for (auto p : list)
        if (p->x == point->x&&p->y == point->y)
            return p;
    return NULL;
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point) const
{//�����������ĸ��������ߵĵ���뵽�����ĵ㼯�ϵ��з���
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
    int x, y; //xb,ybΪĿ�겴λ������
    int time;
    int matched;
    int berthid;
    Cargo():time(1000),berthid(-1),matched(0){}; //��ʼ����λ����Ϊ��-1��-1, �ɴ���ʱ��Ϊ-1

    int findBerth();
};
Cargo* cargos[N][N];

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
    int steps; //��ʱ�����˵�Ŀ��ۿ�����Ĳ���

    //һЩ��ʼ��
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
    berths[id].cargoValues.push(cargoValue);
    berths[id].cargoNum++;
    berths[id].cargoVal+=cargoValue;

    this->cargoValue = 0;
    this->carryState = 0;
    this->targetBerth = -1;

    targetCargo={-1,-1};
}

Point findNearestBerthGrid(const Point& robot, const Point& berth) {
    // ����������ڲ�λ�ڲ���ֱ�ӷ��ػ�����λ��
    if (robot.x >= berth.x && robot.x <= berth.x + 3 &&
        robot.y >= berth.y && robot.y <= berth.y + 3) {
        return robot;
    }

    // ��������˵���λ�ĸ���Ե�����и��ӵľ��룬���ҵ���С����
    int minDistance = INT_MAX;
    Point nearestGrid(0,0);

    // ��������˵���λ�ĸ���Ե�ϵĸ��ӵľ���
    for (int i = berth.x; i <= berth.x + 3; ++i) {
        int distance = abs(robot.x - i) + abs(robot.y - berth.y); // �ϱ�Ե����
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y;
        }

        distance = abs(robot.x - i) + abs(robot.y - (berth.y + 3)); // �±�Ե����
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = i;
            nearestGrid.y = berth.y+3;
        }
    }

    for (int j = berth.y + 1; j < berth.y + 3; ++j) {
        int distance = abs(robot.x - berth.x) + abs(robot.y - j); // ���Ե����
        if (distance < minDistance) {
            minDistance = distance;
            nearestGrid.x = berth.x;
            nearestGrid.y = j;
        }

        distance = abs(robot.x - (berth.x + 3)) + abs(robot.y - j); // �ұ�Ե����
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
        //��ϻ���۸񡢻��ﵽ��λ�ľ������������ľ������ۺ�ѡ��
        int a=1;//����valueȨ��
        int b=1;//�������Ȩ��
        int c=1;//���ﵽ��λ�ľ���Ȩ��
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
    if(carryState==0&&x==targetCargo.first&&x==targetCargo.second){//����û���������ߵ���Ŀ������λ��
        get();
        cout << "get " << id;
        return;
    }
    if(carryState==1&&berths[targetBerth].x<=x&&x<=berths[targetBerth].x+3&&berths[targetBerth].y<=y&&y<=berths[targetBerth].y+3){
        //�����ж��������ߵ��˲�λ�ķ�Χ
        pull(targetBerth);
        cout << "pull " << id;
    }
}

void Robot::planToMove(){
    //move
    //ȷ���������������ĸ������ƶ�
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
        //Ŀ�����Լ������·������ҷ����ȿ��ұߣ�x��y+1�����ٿ��±�(x+1,y)�ĸ���
        if(isCanReach(right)){//����
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(down)){//����
            cout << "move " << id << " " << 3;
            Map[x][y].robotId=-1;
            x++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(up)){//����
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(left)) {//����
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
    }
    else if(end.y<=y&&end.x>x){
        //Ŀ�����Լ������·������·����ȿ��±ߣ�x+1��y�����ٿ����(x,y-1)�ĸ���
        if(isCanReach(down)){//����
            cout << "move " << id << " " << 3;
            Map[x][y].robotId=-1;
            x++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(left)) {//����
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(right)){//����
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(up)){//����
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
    }
    else if(end.y<y&&end.x<=x){
        //Ŀ�����Լ������Ϸ������󷽣��ȿ���ߣ�x+1��y�����ٿ��ϱ�(x,y-1)�ĸ���
        if(isCanReach(left)) {//����
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(up)){//����
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(down)){//����
            cout << "move " << id << " " << 3;
            Map[x][y].robotId=-1;
            x++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(right)){//����
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
    }
    else if(end.y<y&&end.x<=x){
        //Ŀ�����Լ������Ϸ������Ϸ����ȿ��ϱߣ�x+1��y�����ٿ��ұ�(x,y-1)�ĸ���
        if(isCanReach(up)){//����
            cout << "move " << id << " " << 2;
            Map[x][y].robotId=-1;
            x--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(right)){//����
            cout << "move " << id << " " << 0;
            Map[x][y].robotId=-1;
            y++;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(left)) {//����
            cout << "move " << id << " " << 1;
            Map[x][y].robotId=-1;
            y--;
            Map[x][y].robotId=id;
        }
        else if(isCanReach(down)){//����
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
//    std::advance(it, 1); // ����������ǰ�ƶ�һ��λ�ã���������һ��Ԫ��
//    Point* next = *it;
//    if(next->x-x==1){//����
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

    double value(int);
}boats[10];
//void Boat::moveAFrame() {
//    if(shippingTime>0){
//        shippingTime--;
//        if(shippingTime==0){
//            if(targetBerth>=0){ //���ﲴλ
//                berths[targetBerth].receive(Boat::id);
//            }
//            else{ //����vp
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
            for(int i=0;i<berth->BoatNum;i++){
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
    for(int i=0;i<10;i++){
        double v_=value(i);
        if(v_>v){
            v=v_;
            targetBerth=i;
            //shippingTime=berths[targetBerth].transport_time;
        }
    }
    berths[targetBerth].matched=true;

    cout<<"ship "<<Boat::id<<" "<<targetBerth<<endl;

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
        canLoadCargoNum=canLoadCargoValue > berth.cargoNum ? berth.cargoNum : canLoadCargoValue;
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
    cout<<"go "<<Boat::id<<endl;
}


void Berth::load() {
    if(BoatNum>0){
        Boat* boat=&boats[boatsIn[0]];
        int canLoadNum=cargoNum>=loading_speed ? loading_speed : cargoNum;
        int loadNum;
        if(boat->capacity-boat->cargoNum>=canLoadNum){
            loadNum=canLoadNum;
            boat->cargoNum+=canLoadNum;
        }
        else{
            loadNum=boat->capacity-boat->cargoNum;
            boat->cargoNum=boat->capacity;
        }
        cargoNum-=loadNum;
        for(int i=0;i<loadNum;i++) cargoValues.pop();
    }
}

void Berth::receive(int boatId) {
    BoatNum++;
    boatsIn[BoatNum-1]=boatId;
    matched= false;

//    if(BoatNum>1){
//        boats[boatId].state=2;
//    }
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
        if(r.steps<=time){
            if(r.targetBerth==Berth::id){
                cargoValues.push(r.cargoValue);
                cargoNum++;
            }
        }
    }
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
        scanf("%s", line); //��ͼ����
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
        boats[i].targetBerth=i;
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
        cargos[x][y]->x=x;
        cargos[x][y]->y=y;
        cargos[x][y]->value=v;
        cargos[x][y]->time=1000;
        //targetBerth��ȷ��
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
    }
    char okk[100];
    scanf("%s", okk);
    return id;
}

bool compareForRobot(int id1,int id2) {
    //�����������򣬸����������������������
    //���ȣ�û��������ȣ���Ϊ������ܻ���ʧ�����û����ĵ���˭�Ļ����ֵ���˭���ȣ��л����Ҳ��˭�Ļ����ֵ���˭����
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

        //����ָ��
        for(int i=0;i<5;i++){
            boats[i].action();
        }

        //�ۿ�װ�ػ���
        for(int i = 0;i<10;i++){
            berths[i].load();
        }
        puts("OK");
        fflush(stdout);
    }

    return 0;
}