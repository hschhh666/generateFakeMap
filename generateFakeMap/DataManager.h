#pragma once
#include "opencvHeader.h"
#include <random>
//定义数据结构

struct Position {
	double x;
	double y;
};

struct Speed
{
	double vx;
	double vy;
};

struct Acceleration
{
	double ax;
	double ay;
};



//这里定义单个行人的信息
class PedestrianInfo {
public:
	PedestrianInfo() {};
	PedestrianInfo(Position cur, Position tar);

	Position curPosition;//当前位置
	Position lastPosition;//上一时刻位置
	Position initPosition;//初始时刻位置
	Position tarPostion;//目标位置
	Speed curSpeed;//当前速度
	Acceleration curAcc;//当前加速度
	double tarSpeed;//  m/s
	std::list<Position> roadPoints;

private:
	
	double sigma_px = 1.5;
	double sigma_py = 1.5;

};


//这里定义场景的建筑物、尺寸、起点终点等
class SceneStructure {

public:
	SceneStructure() {};
	void InitScene(std::string filename);//初始化地图
	double GetClosestObstacle(double px, double py, double & ox, double & oy);//meter   输入一个点(px,py)，返回地图上距离该点一定范围内最近的一个障碍点(ox,oy）
	void ShowScene(int key, std::string imgname);//地图可视化
	

	bool CoordinateConventer(double, double, int&, int&);//全局坐标与图像坐标的转换，若输入的全局坐标点出现在该地图中则返回true，否则返回false
	bool CoordinateConventer(int, int, double&, double&);//全局坐标与图像坐标的转换，若输入的图像坐标在该图像范围内则返回true，否则返回false；
	
	double leftUpCornerX, leftUpCornerY;//图像左上角对应的真实世界中的全局坐标

	cv::Mat sceneState;//把人也花在场景结构上了

	std::vector<Position> StartEndPositions;//meter 保存着所有的目标点
	
//private:
	int imgSize;//pixel 地图的像素大小
	double sceneSize;//meter 地图对应的实际物理环境大小
	double pixelSize;//meter 每个像素对应的实际物理尺寸
	
	cv::Mat scene;//保存着场景的结构
	
	
	
};
