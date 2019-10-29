#pragma once
#include "DataManager.h"


//���룺�����ṹ�����ˡ���������Ƶ�ʡ�ÿ�ε�����ʾ������ʱ��
class PedestrianSimulator
{
public:
	//simulator�������ǳ����ṹ������������Ƶ���뷽���Ƿ���ӻ���������Լ�������Ƶ�洢λ��
	PedestrianSimulator(std::string sceneFile, std::string pedestrianFile, int VisualRate = 0,std::string videoFile = "");
	void DoSimulation(double delta_t, int timelong);
	~PedestrianSimulator();

private:
	
	void generatePedestrian();//���������ܶȾ���ÿ֡����������ˡ�
	void pathPlanning();//����·���滮
	void updatePositionAndVelocity();//�������˵�λ�����ٶȣ����ж������Ƿ��Ѿ�����Ŀ�ĵ�
	void updateAcceleration();//�������˵ļ��ٶ�
	void updataStateMap();//����״̬��ͼ
	void showCurState();//���ӻ���ǰ����
	void showStateMap();


	int totalFrames;//�ܹ������֡��
	int timeLong;//�ܹ������ʱ��
	double deltaT;//ÿ֡�����ʱ�� ��λ ��
	int curFrame;//��ǰ���浽�ڼ�֡��
	SceneStructure sceneStructure;//��ǰ����
	cv::Mat pedestrianMatrix;//�����ܶȾ���
	cv::Mat StateMap;//״̬��ͼ
	//cv::Mat tmpMatrix;
	std::vector<PedestrianInfo> pedestrians;//������
	int visualRate;//���ӻ����ʣ�1Ϊԭ�ٲ��ţ�2Ϊ2���ٲ��ţ��Դ�����
	cv::VideoWriter video;

	std::random_device rd{};
	std::mt19937 gen{ rd() };


};

