#include "PedestrianSimulator.h"



PedestrianSimulator::PedestrianSimulator(std::string sceneFile, std::string pedestrianFile, int VisualRate, std::string videoFile)
{

	visualRate = VisualRate;
	sceneStructure.InitScene(sceneFile);
	cv::FileStorage storage(pedestrianFile, cv::FileStorage::READ);
	storage["matrix"] >> pedestrianMatrix;//读取场景结构
	for (int i = 0; i < pedestrianMatrix.rows; i++)
	{
		for (int j = 0; j < pedestrianMatrix.cols; j++) {
			std::cout << pedestrianMatrix.at<double>(i, j) << " ";//读取人流密度矩阵
		}
		std::cout << std::endl;
	}
	
	if(videoFile!="")
		if (!video.open(videoFile, CV_FOURCC('X', 'V', 'I', 'D'), 20, cv::Size(500, 500))) {
			std::cout << "Cannot open video file " << videoFile << ", program exit.\n";
			exit(-4);
		}

	//tmpMatrix = cv::Mat::zeros(cv::Size(pedestrianMatrix.cols, pedestrianMatrix.cols), CV_32F);
	StateMap = cv::Mat::ones(cv::Size(sceneStructure.sceneState.rows, sceneStructure.sceneState.cols), CV_8UC1)*255;//初始化状态地图

}

void PedestrianSimulator::DoSimulation(double delta_t, int timelong)
{
	deltaT = delta_t;
	timeLong = timelong;
	totalFrames = timeLong / deltaT;
	curFrame = 0;
	
	while (curFrame < totalFrames) {
		generatePedestrian();
		updatePositionAndVelocity();
		updateAcceleration();
		showCurState();
		updataStateMap();
		std::cout << "frame : " << curFrame << std::endl;
		curFrame++;
	}

	cv::imshow("stateMap", StateMap);
	cv::waitKey(0);

	//for (int i = 0; i < tmpMatrix.rows; i++)
	//{
	//	for (int j = 0; j < tmpMatrix.cols; j++)
	//		printf("%.1f ", tmpMatrix.at<float>(i, j) * 60 / timeLong);
	//	std::cout << std::endl;
	//}
	//std::cout << std::endl << std::endl << std::endl;
	//for (int i = 0; i < pedestrianMatrix.rows; i++)
	//{
	//	for (int j = 0; j < pedestrianMatrix.cols; j++) {
	//		std::cout << pedestrianMatrix.at<double>(i, j) << " ";
	//	}
	//	std::cout << std::endl;
	//}
		
}

void PedestrianSimulator::generatePedestrian()
{
	
	std::uniform_real_distribution<double> uniform(0, 10);
	double value;
	double randomValue;
	double prob;
	double x, y;
	
	for(int i=0;i<pedestrianMatrix.rows;i++)
		for (int j = 0; j < pedestrianMatrix.cols; j++) {
			value = pedestrianMatrix.at<double>(i, j);

			prob = (deltaT / 60)*value;
			if (prob*10 < uniform(gen)) continue;//按照一定概率生成行人

			//tmpMatrix.at<float>(i, j)++;

			double tmp1, tmp2, dis;
			do {
				PedestrianInfo onePedestrian(sceneStructure.StartEndPositions[i], sceneStructure.StartEndPositions[j]);
				dis = sceneStructure.GetClosestObstacle(onePedestrian.curPosition.x, onePedestrian.curPosition.y, tmp1, tmp2);//行人不能出生在障碍物上
				if(dis>=0.1)
					pedestrians.push_back(onePedestrian);

			} while (dis<0.1);
		}
}

void PedestrianSimulator::pathPlanning()
{
}

void PedestrianSimulator::updatePositionAndVelocity()
{

	for (int i = 0; i < pedestrians.size(); i++) {//判断行人有没有到达终点，如果到达，则删除
		double dis = (pedestrians[i].curPosition.x - pedestrians[i].tarPostion.x)*(pedestrians[i].curPosition.x - pedestrians[i].tarPostion.x)
			+ (pedestrians[i].curPosition.y - pedestrians[i].tarPostion.y)*(pedestrians[i].curPosition.y - pedestrians[i].tarPostion.y);
		dis = sqrt(dis);
		if (dis < 1)
		{
			pedestrians.erase(pedestrians.begin() + i);
			i--;
		}
	}

	//更新行人的位置和速度
	for (auto &p : pedestrians) {
		p.lastPosition.x = p.curPosition.x;
		p.lastPosition.y = p.curPosition.y;

		p.curPosition.x += p.curSpeed.vx * deltaT;
		p.curPosition.y += p.curSpeed.vy * deltaT;

		p.curSpeed.vx += p.curAcc.ax * deltaT;
		p.curSpeed.vy += p.curAcc.ay * deltaT;
	}


}

void PedestrianSimulator::updateAcceleration()
{

	//更新行人的加速度
	for (auto &p : pedestrians) {
		Speed targetSpeed;
		targetSpeed.vx = p.tarPostion.x - p.curPosition.x;
		targetSpeed.vy = p.tarPostion.y - p.curPosition.y;
		double lenth = sqrt(targetSpeed.vx*targetSpeed.vx + targetSpeed.vy*targetSpeed.vy);
		targetSpeed.vx /= lenth;
		targetSpeed.vy /= lenth;
		targetSpeed.vx *= p.tarSpeed;
		targetSpeed.vy *= p.tarSpeed;

		Acceleration a1;//加速度中的第一项，即目的地造成的加速度
		a1.ax = (targetSpeed.vx - p.curSpeed.vx) / 1;
		a1.ay = (targetSpeed.vy - p.curSpeed.vy) / 1;



		Acceleration a2;//加速度第二项，即周围行人对当前行人的斥力
		a2.ax = 0;
		a2.ay = 0;
		double Vab0 = 3;//这个参数论文里是2.1
		double sigma = 0.8;//这个参数论文里是0.3
		for (auto q : pedestrians) {

			double disTwoPerson = (p.curPosition.x - q.curPosition.x)*(p.curPosition.x - q.curPosition.x) + (p.curPosition.y - q.curPosition.y)*(p.curPosition.y - q.curPosition.y);
			disTwoPerson = sqrt(disTwoPerson);
			double tmp1, tmp2;
			double tmpDeltaT = 2.0;
			tmp1 = (p.curPosition.x - q.curPosition.x - q.curSpeed.vx*tmpDeltaT);
			tmp2 = (p.curPosition.y - q.curPosition.y - q.curSpeed.vy*tmpDeltaT);
			double nextTimeDisTwoPerson = sqrt(tmp1*tmp1 + tmp2 * tmp2);
			double deltaPerson = q.curSpeed.vx*tmpDeltaT * q.curSpeed.vx*tmpDeltaT + q.curSpeed.vy*tmpDeltaT * q.curSpeed.vy*tmpDeltaT;
			double two_b = sqrt(pow(disTwoPerson + nextTimeDisTwoPerson, 2) - deltaPerson);

			double Vab = Vab0 * exp(-two_b / (2 * sigma));
			double dir_x, dir_y;
			dir_x = p.curPosition.x - q.curPosition.x;
			dir_y = p.curPosition.y - q.curPosition.y;
			double lenth = sqrt(dir_x*dir_x + dir_y * dir_y);
			if (lenth < 0.0001) continue;

			dir_x /= lenth;
			dir_y /= lenth;

			double coefficient = 1;
			double cosTheta = p.curSpeed.vx * (q.curPosition.x - p.curPosition.x) + p.curSpeed.vy *(q.curPosition.y - p.curPosition.y);//计算别人是不是在我的视野范围内
			cosTheta /= (sqrt(p.curSpeed.vx*p.curSpeed.vx + p.curSpeed.vy*p.curSpeed.vy) * disTwoPerson);
			double Theta = acos(cosTheta) * 180 / CV_PI;
			if (Theta > 100)//对于我视野范围内100*2度之外的人，视而不见
				coefficient = 0.2;

			a2.ax += dir_x * Vab * coefficient;
			a2.ay += dir_y * Vab * coefficient;

		}


		Acceleration a3;//加速度第三项，即墙壁对当前行人的斥力
		a3.ax = 0;
		a3.ay = 0;
		double UaB0 = 10;//这个参数论文里是10m
		double R = 0.6;//这个参数论文里是0.2m
		double ox, oy,dis;
		dis = sceneStructure.GetClosestObstacle(p.curPosition.x, p.curPosition.y, ox, oy);
		if (dis < 10) {
			double UaB = UaB0 * exp(-dis / R);

			double dir_x, dir_y;
			dir_x = p.curPosition.x - ox;
			dir_y = p.curPosition.y - oy;
			double lenth = sqrt(dir_x*dir_x + dir_y * dir_y);
			dir_x /= lenth;
			dir_y /= lenth;

			a3.ax = dir_x * UaB;
			a3.ay = dir_y * UaB;

		}

		/*人在靠近墙壁，且目的地位于墙壁同一侧时，行人速度会大幅度降低，因为墙的斥力抵消了目的地对人的大部分吸引力。我在这里做了相应的修改，即当检测到行人速度很低，且目的地引力被墙壁斥力抵消时，就将目的地对人的吸引力全加到行人目前的前进方向上，这样就保证了即使人靠近墙壁，也不会出现原来算法中的大幅度减速问题。*/
		double curSpeed = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));//当前的速度大小，标量
		double curAccPersonAndObst = sqrt(pow(a1.ax + a3.ax, 2) + pow(a1.ay + a3.ay, 2));//当前目的地引力与墙斥力向量和的大小，标量
		double curAccPerson = sqrt(pow(a1.ax, 2) + pow(a1.ay, 2));//当前目的地对行人的引力大小，标量

		if (curSpeed < p.tarSpeed*0.9 &&   curAccPersonAndObst< curAccPerson*0.8) {//如果行人速度低，且墙壁斥力抵消了大部分目的地引力
			double lenth = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));//
			double curDirx = p.curSpeed.vx / lenth;//当前前进方向
			double curDiry = p.curSpeed.vy / lenth;//当前前进方向，注意是单位向量
			double horizonAcc = a1.ax*curDirx + a1.ay*curDiry;//当前目的地引力在行人前进方向上的分量
			a1.ax -= horizonAcc   * curDirx;//
			a1.ay -= horizonAcc   * curDiry;//先减掉目的地引力水平方向分量，现在目的地引力只有垂直于墙壁的分量了
			a1.ax += curAccPerson * curDirx;
			a1.ay += curAccPerson * curDiry;//现在，平行于墙壁的力的大小直接等于目的地引力大小了
		}

		p.curAcc.ax = a1.ax + a2.ax + a3.ax;
		p.curAcc.ay = a1.ay + a2.ay + a3.ay;
	 }
}

void PedestrianSimulator::updataStateMap()
{
	int ix, iy;
	double x, y;
	for (auto p : pedestrians) {
		x = p.curPosition.x;
		y = p.curPosition.y;

		if (!sceneStructure.CoordinateConventer(x, y, ix, iy)) continue;

		if (StateMap.at<uchar>(cv::Point(ix, iy))>1)
			StateMap.at<uchar>(cv::Point(ix, iy)) -= 2;
	}
}

void PedestrianSimulator::showCurState()
{
	cv::Mat state = sceneStructure.sceneState.clone();
	double curTime = curFrame * deltaT;
	char word[200];
	sprintf(word, "Simulation time = %.1f s", curTime);
	int x, y;


	for (auto p : pedestrians) {
		if (sceneStructure.CoordinateConventer(p.curPosition.x, p.curPosition.y, x, y)) {
			cv::circle(state, cv::Point(x, y), 2, cv::Scalar(255, 0, 0),-1);
		}
	}

	cv::putText(state, word, cv::Point(5, 15), 0, 0.5, cv::Scalar(0, 0, 255));

	if (visualRate) {
		cv::imshow("Simulator", state);
		cv::waitKey(deltaT * 1000 / visualRate);
	}

	if(video.isOpened())
		video.write(state);

	
}


PedestrianSimulator::~PedestrianSimulator()
{
}