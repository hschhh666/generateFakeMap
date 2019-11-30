#pragma once
#include "opencvHeader.h"
#include <random>
//�������ݽṹ

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



//���ﶨ�嵥�����˵���Ϣ
class PedestrianInfo {
public:
	PedestrianInfo() {};
	PedestrianInfo(Position cur, Position tar);

	Position curPosition;//��ǰλ��
	Position lastPosition;//��һʱ��λ��
	Position initPosition;//��ʼʱ��λ��
	Position tarPostion;//Ŀ��λ��
	Speed curSpeed;//��ǰ�ٶ�
	Acceleration curAcc;//��ǰ���ٶ�
	double tarSpeed;//  m/s
	std::list<Position> roadPoints;

private:
	
	double sigma_px = 1.5;
	double sigma_py = 1.5;

};


//���ﶨ�峡���Ľ�����ߴ硢����յ��
class SceneStructure {

public:
	SceneStructure() {};
	void InitScene(std::string filename);//��ʼ����ͼ
	double GetClosestObstacle(double px, double py, double & ox, double & oy);//meter   ����һ����(px,py)�����ص�ͼ�Ͼ���õ�һ����Χ�������һ���ϰ���(ox,oy��
	void ShowScene(int key, std::string imgname);//��ͼ���ӻ�
	

	bool CoordinateConventer(double, double, int&, int&);//ȫ��������ͼ�������ת�����������ȫ�����������ڸõ�ͼ���򷵻�true�����򷵻�false
	bool CoordinateConventer(int, int, double&, double&);//ȫ��������ͼ�������ת�����������ͼ�������ڸ�ͼ��Χ���򷵻�true�����򷵻�false��
	
	double leftUpCornerX, leftUpCornerY;//ͼ�����ϽǶ�Ӧ����ʵ�����е�ȫ������

	cv::Mat sceneState;//����Ҳ���ڳ����ṹ����

	std::vector<Position> StartEndPositions;//meter ���������е�Ŀ���
	
//private:
	int imgSize;//pixel ��ͼ�����ش�С
	double sceneSize;//meter ��ͼ��Ӧ��ʵ����������С
	double pixelSize;//meter ÿ�����ض�Ӧ��ʵ������ߴ�
	
	cv::Mat scene;//�����ų����Ľṹ
	
	
	
};
