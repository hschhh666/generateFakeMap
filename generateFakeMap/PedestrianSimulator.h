#pragma once
#include "DataManager.h"


//输入：场景结构、行人、行人生成频率、每次迭代表示的物理时长
class PedestrianSimulator
{
public:
	//simulator的输入是场景结构、场景中人流频率与方向、是否可视化仿真过程以及方阵视频存储位置
	PedestrianSimulator(std::string sceneFile, std::string pedestrianFile, int VisualRate = 0,std::string videoFile = "");
	void DoSimulation(double delta_t, int timelong);
	~PedestrianSimulator();

private:
	
	void generatePedestrian();//根据人流密度矩阵，每帧随机生成行人。
	void pathPlanning();//行人路径规划
	void updatePositionAndVelocity();//更新行人的位置与速度，并判断行人是否已经到达目的地
	void updateAcceleration();//更新行人的加速度
	void updataStateMap();//更新状态地图
	void showCurState();//可视化当前场景
	void showStateMap();


	int totalFrames;//总共仿真的帧数
	int timeLong;//总共仿真的时长
	double deltaT;//每帧代表的时间 单位 秒
	int curFrame;//当前仿真到第几帧了
	SceneStructure sceneStructure;//当前场景
	cv::Mat pedestrianMatrix;//人流密度矩阵
	cv::Mat StateMap;//状态地图
	//cv::Mat tmpMatrix;
	std::vector<PedestrianInfo> pedestrians;//行人们
	int visualRate;//可视化速率，1为原速播放，2为2倍速播放，以此类推
	cv::VideoWriter video;

	std::random_device rd{};
	std::mt19937 gen{ rd() };


};

