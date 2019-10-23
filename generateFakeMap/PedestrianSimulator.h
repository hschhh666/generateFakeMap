#pragma once
#include "DataManager.h"


//输入：场景结构、行人、行人生成频率、每次迭代表示的物理时长
class PedestrianSimulator
{
public:
	PedestrianSimulator(std::string sceneFile, std::string pedestrianFile, std::string videoFile = "");
	void DoSimulation(double delta_t, int timelong);
	~PedestrianSimulator();

private:
	
	void generatePedestrian();
	void updatePositionAndVelocity();//这里注意要先判断行人是否已经到达目的地
	void updateAcceleration();
	void showCurState();


	int totalFrames;//总共仿真的帧数
	int timeLong;//总共仿真的时长
	double deltaT;//每帧代表的时间 单位 秒
	int curFrame;//当前仿真到第几帧了
	SceneStructure sceneStructure;
	cv::Mat pedestrianMatrix;
	std::vector<PedestrianInfo> pedestrians;

	cv::VideoWriter video;

};

