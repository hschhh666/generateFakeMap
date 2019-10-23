#pragma once
#include "DataManager.h"


//���룺�����ṹ�����ˡ���������Ƶ�ʡ�ÿ�ε�����ʾ������ʱ��
class PedestrianSimulator
{
public:
	PedestrianSimulator(std::string sceneFile, std::string pedestrianFile, std::string videoFile = "");
	void DoSimulation(double delta_t, int timelong);
	~PedestrianSimulator();

private:
	
	void generatePedestrian();
	void updatePositionAndVelocity();//����ע��Ҫ���ж������Ƿ��Ѿ�����Ŀ�ĵ�
	void updateAcceleration();
	void showCurState();


	int totalFrames;//�ܹ������֡��
	int timeLong;//�ܹ������ʱ��
	double deltaT;//ÿ֡�����ʱ�� ��λ ��
	int curFrame;//��ǰ���浽�ڼ�֡��
	SceneStructure sceneStructure;
	cv::Mat pedestrianMatrix;
	std::vector<PedestrianInfo> pedestrians;

	cv::VideoWriter video;

};

