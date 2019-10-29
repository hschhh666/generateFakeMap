#include <iostream>
#include "define.h"

CvRect box1;
void MouseCallback1(int event, int x, int y, int flag, void * param)
{
	cv::Mat scene = *((cv::Mat *) param);

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		box1 = cvRect(x, y, 0, 0);
		std::cout << "CV_EVENT_LBUTTONDOWN\n";
		break;
	case CV_EVENT_MOUSEMOVE:
		box1.width = x - box1.x;
		box1.height = y - box1.y;
		std::cout << "CV_EVENT_MOUSEMOVE\n";
		break;
	case CV_EVENT_LBUTTONUP:
		box1.width = x - box1.x;
		box1.height = y - box1.y;
		std::cout << "CV_EVENT_LBUTTONUP\n";
		cv::rectangle(scene, box1, cv::Scalar(0, 0, 0));
		break;
	}

	
}

/*算法流程
初始化部分：
1. 定义环境的结构，即设定起点终点、建筑物位置等。写成一个类，每个实例表示一个场景
2. 设定每次迭代表示的真实物理时间，可以考虑0.1s
3. 设定仿真器的运行时长，即总共仿真多少帧
4. 设定每个起点生成行人的频率 如10人/min， 设定行人的终点

循环运算部分：
1. 生成行人
2. 更新所有行人的位置和速度
3. 更新所有行人的加速度
*/


//主函数里定义每次迭代表示的物理时长、迭代次数、生成人的频率等
int main(int argc, char ** argv)
{
	if (argc != 2) {
		std::cout << "No config file, press any key to exit.\n";
		getchar();
		return(-2);
	}
	inifile::IniFile config;
	if (!config.load(std::string(argv[1]))) {
		printf("press any key to exit.\n");
		getchar();
		return (-3);
	}

	std::string sceneFile, pedestrianMatrix, video="";
	int visualRate ,simulationTime;
	config.getValue("param", "sceneFile", sceneFile);
	config.getValue("param", "pedestrianMatrix", pedestrianMatrix);
	config.getValue("param", "video", video);
	config.getIntValue("param", "visualRate", visualRate);
	config.getIntValue("param", "simulationTime", simulationTime);


	PedestrianSimulator pedestrianSim(sceneFile, pedestrianMatrix,  visualRate, video);//simulator的输入是场景结构、场景中人流频率与方向、是否可视化仿真过程以及方阵视频存储位置

	pedestrianSim.DoSimulation(0.1, simulationTime);//仿真器主函数，第一个参数是每帧代表的时长（秒），第二个参数是总共仿真多少秒




    std::cout << "Hello World!\n"; 

}

