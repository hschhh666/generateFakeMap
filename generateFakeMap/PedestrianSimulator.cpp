#include "PedestrianSimulator.h"



PedestrianSimulator::PedestrianSimulator(std::string sceneFile, std::string satelliteFile,std::string pedestrianFile, int VisualRate, std::string videoFile,std::string outputFile)
{

	visualRate = VisualRate;
	sceneStructure.InitScene(sceneFile,satelliteFile);//读取场景结构和卫星图
	cv::FileStorage storage(pedestrianFile, cv::FileStorage::READ);
	storage["matrix"] >> pedestrianMatrix;//读取人流密度矩阵
	for (int i = 0; i < pedestrianMatrix.rows; i++)
	{
		for (int j = 0; j < pedestrianMatrix.cols; j++) {
			std::cout << pedestrianMatrix.at<double>(i, j) << " ";//输出人流密度矩阵
		}

		std::cout << std::endl;
	}
	
	if(videoFile!="")
		if (!video.open(videoFile, CV_FOURCC('X', 'V', 'I', 'D'), 10, cv::Size(sceneStructure.imgSize, sceneStructure.imgSize))) {
			std::cout << "Cannot open video file " << videoFile << ", program exit.\n";
			exit(-4);
		}

	tmpMatrix = cv::Mat::zeros(cv::Size(pedestrianMatrix.cols, pedestrianMatrix.cols), CV_32F);
	StateMap = cv::Mat::zeros(cv::Size(sceneStructure.sceneState.rows, sceneStructure.sceneState.cols), CV_32FC(9));//初始化状态地图
	outputDir = outputFile;
	
	double kernelSize = 0.5;//计算以人为中心，kernelSize米范围内
	double pixelSize = sceneStructure.pixelSize;
	int kernelPixelSize = 2*(kernelSize / pixelSize) + 1;
	double theta = 0.2;//高斯核的衰减半径，theta时衰减到0.6
	
	stateMapKernal = cv::Mat::zeros(cv::Size(kernelPixelSize, kernelPixelSize), CV_32F);
	for (int i = 0;i< kernelPixelSize;i++)
	{ 
		for (int j = 0; j < kernelPixelSize; j++) {
			int x = j - (int)(kernelPixelSize / 2);
			int y = (int)(kernelPixelSize / 2) - i;
			float dis = sqrt((double)(x * x + y * y));
			dis = dis * pixelSize;
			float value = exp(-dis * dis/(2*theta*theta));
			stateMapKernal.at<float>(i, j) = value;
		}

	}
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
		if (curFrame%200 == 0)
			std::cout << "frame : " << curFrame << "/"<<totalFrames<<std::endl;
		curFrame++;
	}

	if (outputDir == "")
		showStateMap();
	else
		saveStateMap();
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

			tmpMatrix.at<float>(i, j)++;

			double tmp1, tmp2, dis1,dis2,dis3;
			do {

				std::vector<Position> roadPoints;
				roadPoints.push_back(sceneStructure.StartEndPositions[i]);
				int midpoint = sceneStructure.HiddenMatrix.at<double>(i, j);
				if (midpoint >= 0)
					roadPoints.push_back(sceneStructure.HiddenPositions.at(midpoint));
				roadPoints.push_back(sceneStructure.StartEndPositions[j]);

				//PedestrianInfo onePedestrian(sceneStructure.StartEndPositions[i], sceneStructure.StartEndPositions[j]);
				PedestrianInfo onePedestrian(roadPoints);
				dis1 = sceneStructure.GetClosestObstacle(onePedestrian.curPosition.x, onePedestrian.curPosition.y, tmp1, tmp2);//行人不能出生在障碍物上
				dis2 = sceneStructure.GetClosestObstacle(onePedestrian.tarPostion.x, onePedestrian.tarPostion.y, tmp1, tmp2);//目的地不可以在障碍物上
				dis3 = 10000;
				if(midpoint >=0 )
					dis3 = sceneStructure.GetClosestObstacle(onePedestrian.endPosition.x, onePedestrian.endPosition.y, tmp1, tmp2);//中间目的地不可以在障碍物上
				
				if(dis1>=0.1 && dis2 >=0.1 && dis3 >= 0.1)
					pedestrians.push_back(onePedestrian);

			} while (dis1<0.1 || dis2<0.1 || dis3<0.1);

		}

}

void PedestrianSimulator::pathPlanning()
{
}

void PedestrianSimulator::updatePositionAndVelocity()
{

	for (int i = 0; i < pedestrians.size(); i++) {//判断行人有没有到达终点，如果到达，则删除
		double dis1 = (pedestrians[i].curPosition.x - pedestrians[i].tarPostion.x)*(pedestrians[i].curPosition.x - pedestrians[i].tarPostion.x)
			+ (pedestrians[i].curPosition.y - pedestrians[i].tarPostion.y)*(pedestrians[i].curPosition.y - pedestrians[i].tarPostion.y);
		dis1 = sqrt(dis1);//行人当前位置距离target位置的距离
		double tmp1, tmp2;
		double dis2 = sceneStructure.GetClosestObstacle(pedestrians[i].curPosition.x, pedestrians[i].curPosition.y, tmp1, tmp2);
		if (fabs(dis2) < 0.1){//如果行人走着走着走到障碍物上了，那就删掉他！
			pedestrians.erase(pedestrians.begin() + i);
			i--;
			continue;
		}
		if (dis1 < 1) {//如果行人距离target位置很近了
			if (pedestrians.at(i).endPosition.x >= 0) {//判断一下是不是终点，如果不是终点，则把当前的target点改为终点
				pedestrians.at(i).tarPostion.x = pedestrians.at(i).endPosition.x;
				pedestrians.at(i).tarPostion.y = pedestrians.at(i).endPosition.y;
				pedestrians.at(i).endPosition.x = -1;//很蠢，如果访问过中间目的地了，就把endPosition置为负数
			}
			else {//如果是终点了，删掉行人
				pedestrians.erase(pedestrians.begin() + i);
				i--;
				continue;
			}
		}

		
	}

	//更新行人的位置和速度
	for (auto &p : pedestrians) {
		p.lastPosition.x = p.curPosition.x;
		p.lastPosition.y = p.curPosition.y;

		double curSpeed = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));
		if (curSpeed / p.tarSpeed > 1.3) {//限制速度上限为目标速度的1.3倍
			p.curSpeed.vx *= (1.3*p.tarSpeed / curSpeed);
			p.curSpeed.vy *= (1.3*p.tarSpeed / curSpeed);

		}

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
		double Vab0 = 2.0;//这个参数论文里是2.1
		double sigma = 0.3;//这个参数论文里是0.3
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
		double R = 0.6;//这个参数论文里是0.2m，我之前用的是0.6
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

		if (curSpeed < p.tarSpeed*2 &&   curAccPersonAndObst< curAccPerson*0.5) {//如果行人速度低，且墙壁斥力抵消了大部分目的地引力
			double lenth = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));//
			double curDirx = p.curSpeed.vx / lenth;//当前前进方向
			double curDiry = p.curSpeed.vy / lenth;//当前前进方向，注意是单位向量
			double horizonAcc = a1.ax*curDirx + a1.ay*curDiry;//当前目的地引力在行人前进方向上的分量
			a1.ax -= horizonAcc   * curDirx;//
			a1.ay -= horizonAcc   * curDiry;//先减掉目的地引力水平方向分量，现在目的地引力只有垂直于墙壁的分量了
			a1.ax += curAccPerson * curDirx;
			a1.ay += curAccPerson * curDiry;//现在，平行于墙壁的力的大小直接等于目的地引力大小了
			a3.ax = a3.ay = 0;//老子不要墙壁的斥力了！
		}

		p.curAcc.ax = a1.ax + a2.ax + a3.ax;
		p.curAcc.ay = a1.ay + a2.ay + a3.ay;
	 }
}

void PedestrianSimulator::updataStateMap()
{
	int ix, iy;
	double cx, cy,lx,ly,dis,theta;
	for (auto p : pedestrians) {
		cx = p.curPosition.x;
		cy = p.curPosition.y;
		lx = p.lastPosition.x;
		ly = p.lastPosition.y;
		dis = sqrt(pow(cx - lx, 2) + pow(cy - ly, 2));
		if (dis < 0.01) continue;

		theta = acos((cx - lx) / dis);
		theta = theta * 180 / CV_PI;
		if (cy - ly < 0)
			theta = 360 - theta;
		theta += 22.5;
		theta = theta - ((int)(theta / 360.0)) * 360;
		int dir = theta / 45;

		double speed = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));
		if (speed < 0.1)//速度小于某个阈值，则认为人没动，就不处理
			continue;

		//状态如何定义：状态定义为四个方向，上下左右，用四个通道来存储。
		double dirX = p.curSpeed.vx / speed;//人在X方向上的分量
		double dirY = p.curSpeed.vy / speed;//人在y方向上的分量
		int kernelPixelSize = stateMapKernal.cols;
		if (sceneStructure.CoordinateConventer(cx, cy, ix, iy)) {

			for (int i = 0; i < kernelPixelSize; i++)
				for (int j = 0; j < kernelPixelSize; j++) {
					float value = stateMapKernal.at<float>(i, j);
					int tmp_y = i - (kernelPixelSize / 2);
					int tmp_x = j - (kernelPixelSize / 2);
					if (ix + tmp_x >= sceneStructure.imgSize || ix + tmp_x < 0 || iy + tmp_y >= sceneStructure.imgSize || iy + tmp_y < 0)
						continue;
					if (dirX > 0)
						StateMap.at<cv::Vec<float, 9>>(cv::Point(ix + tmp_x, iy + tmp_y))[0] += (dirX * value);//以人为中心，做高斯衰减
					else
						StateMap.at<cv::Vec<float, 9>>(cv::Point(ix + tmp_x, iy + tmp_y))[2] += ((-dirX) * value);
					if (dirY > 0)
						StateMap.at<cv::Vec<float, 9>>(cv::Point(ix + tmp_x, iy + tmp_y))[1] += (dirY * value);
					else
						StateMap.at<cv::Vec<float, 9>>(cv::Point(ix + tmp_x, iy + tmp_y))[3] += ((-dirY) * value);

					StateMap.at<cv::Vec<float, 9>>(cv::Point(ix + tmp_x, iy + tmp_y))[8] += value;
				}
		}

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

void PedestrianSimulator::showStateMap()
{
	cv::Mat channels[9];
	cv::split(StateMap, channels);

	double maxValue, minValue;
	cv::minMaxIdx(channels[8], &minValue, &maxValue);
	for(int i=0;i<StateMap.rows;i++)
		for (int j = 0; j < StateMap.cols; j++) {

			for (int k = 0; k < 8; k++) {
				if(channels[8].at<float>(i, j) >0.5)
					channels[k].at<float>(i, j) = channels[k].at<float>(i, j) / channels[8].at<float>(i, j);
			}
			channels[8].at<float>(i, j) =channels[8].at<float>(i, j) / maxValue;
			for (int k = 0; k < 8; k++) {
				channels[k].at<float>(i, j) = channels[k].at<float>(i, j) *channels[8].at<float>(i, j);
				channels[k].at<float>(i, j) = 1 - channels[k].at<float>(i, j);
			}
			channels[8].at<float>(i, j) = 1 - channels[8].at<float>(i, j);
		}
	std::string windowName[10] = { "To Right", "To UpRight", "To Up", "To UpLeft", "To Left", "To DownLeft", "To Down", "To DownRight","ToTal Count" };

	double arrowLenth = 50;
	double angel45 = 45 * CV_PI / 180;
	for (int i = 0; i < 9; i++) {
		//cv::putText(channels[i], windowName[i], cv::Point(20, 40),3 , 1, cv::Scalar(255),1);
		double x1 = arrowLenth * 1.1;
		double y1 = arrowLenth * 1.1;
		double x2 = arrowLenth * cos(angel45*i);
		double y2 = arrowLenth * sin(angel45*i);
		x2 += x1;
		y2 = y1 - y2;
		if (i == 8) {
			cv::putText(channels[i], windowName[i], cv::Point(20, 40), 3, 1, cv::Scalar(0), 1); continue;
		}
		cv::arrowedLine(channels[i], cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0), 2);
	}

	cv::Mat visualMap;
	visualMap = cv::Mat::zeros(StateMap.rows * 3, StateMap.cols * 3, CV_32F);

	cv::Mat ROI[9];

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			ROI[i * 3 + j] = visualMap(cv::Rect(j*StateMap.cols, i*StateMap.rows, StateMap.cols, StateMap.rows));

		}


	channels[3].copyTo(ROI[0]); channels[2].copyTo(ROI[1]); channels[1].copyTo(ROI[2]);
	channels[4].copyTo(ROI[3]); channels[8].copyTo(ROI[4]); channels[0].copyTo(ROI[5]);
	channels[5].copyTo(ROI[6]); channels[6].copyTo(ROI[7]); channels[7].copyTo(ROI[8]);

	int lineWidth = 5;
	cv::Scalar color(0);
	cv::line(visualMap, cv::Point(StateMap.cols, 0), cv::Point(StateMap.cols, StateMap.rows * 3), color, lineWidth);
	cv::line(visualMap, cv::Point(2 * StateMap.cols, 0), cv::Point(2 * StateMap.cols, StateMap.rows * 3), color, lineWidth);
	cv::line(visualMap, cv::Point(0, StateMap.rows), cv::Point(StateMap.cols * 3, StateMap.rows), color, lineWidth);
	cv::line(visualMap, cv::Point(0, 2 * StateMap.rows), cv::Point(StateMap.cols * 3, 2 * StateMap.rows), color, lineWidth);

	cv::resize(visualMap, visualMap, cv::Size(1200, 1200));

	visualMap.convertTo(visualMap, CV_8U, 255);
	cv::imshow("statemap", visualMap);
	cv::waitKey(0);


}

void PedestrianSimulator::saveStateMap()
{
	cv::Mat channels[9];
	cv::split(StateMap, channels);
	cv::FileStorage storage(outputDir + ".yml", cv::FileStorage::WRITE);

	storage << "simulationTime" << timeLong;
	storage << "originPedestrianMatrix" << pedestrianMatrix;
	storage << "generatedPedestrianMatrix" << tmpMatrix * 60 / timeLong;

	storage << "stateMap" << channels[8];
	storage << "toRight"  << channels[0];
	storage << "toUp"     << channels[1];
	storage << "toLeft"   << channels[2];
	storage << "toDown"   << channels[3];

	storage.release();
	printf("Write state map to %s\n", outputDir + ".yml");

	//以下为可视化部分
	cv::Mat colorStateMap = cv::Mat::zeros(StateMap.rows, StateMap.cols, CV_8UC3);//该mat保存的是彩色状态图
	cv::Mat toRight = cv::Mat::zeros(StateMap.rows, StateMap.cols, CV_8UC3);//该mat保存的是右通道彩色状态图
	cv::Mat toLeft = cv::Mat::zeros(StateMap.rows, StateMap.cols, CV_8UC3);//该mat保存的是左通道彩色状态图
	cv::Mat toUp = cv::Mat::zeros(StateMap.rows, StateMap.cols, CV_8UC3);//该mat保存的是上通道彩色状态图
	cv::Mat toDown = cv::Mat::zeros(StateMap.rows, StateMap.cols, CV_8UC3);//该mat保存的是下通道彩色状态图

	cv::Mat alpha(cv::Size(colorStateMap.cols, colorStateMap.rows), CV_8UC1, cv::Scalar(255));//stateMap的alpha通道

	double peopleNum, toLeftPeopleNum, toRightPeopleNum, toUpPeopleNum, toDownPeopleNum;
	double visThreshold = 70;//可视化的阈值，当人数超过该阈值时，赋予相同的颜色
	double singleVisThreshold = visThreshold / 2;//单个通道的可视化阈值
	for (int i = 0; i < StateMap.rows; i++)
		for (int j = 0; j < StateMap.cols; j++) {
			peopleNum = channels[8].at<float>(i, j);
			toRightPeopleNum = channels[0].at<float>(i, j);
			toUpPeopleNum = channels[1].at<float>(i, j);
			toLeftPeopleNum = channels[2].at<float>(i, j);
			toDownPeopleNum = channels[3].at<float>(i, j);

			peopleNum = peopleNum / (float(timeLong) / 60.0);//per minutes
			peopleNum = peopleNum > visThreshold ? visThreshold : peopleNum;
			peopleNum = peopleNum * 255 / visThreshold;

			toRightPeopleNum = toRightPeopleNum / (float(timeLong) / 60.0);//per minutes
			toRightPeopleNum = toRightPeopleNum > singleVisThreshold ? singleVisThreshold : toRightPeopleNum;
			toRightPeopleNum = toRightPeopleNum * 255 / singleVisThreshold;

			toUpPeopleNum = toUpPeopleNum / (float(timeLong) / 60.0);//per minutes
			toUpPeopleNum = toUpPeopleNum > singleVisThreshold ? singleVisThreshold : toUpPeopleNum;
			toUpPeopleNum = toUpPeopleNum * 255 / singleVisThreshold;

			toLeftPeopleNum = toLeftPeopleNum / (float(timeLong) / 60.0);//per minutes
			toLeftPeopleNum = toLeftPeopleNum > singleVisThreshold ? singleVisThreshold : toLeftPeopleNum;
			toLeftPeopleNum = toLeftPeopleNum * 255 / singleVisThreshold;

			toDownPeopleNum = toDownPeopleNum / (float(timeLong) / 60.0);//per minutes
			toDownPeopleNum = toDownPeopleNum > singleVisThreshold ? singleVisThreshold : toDownPeopleNum;
			toDownPeopleNum = toDownPeopleNum * 255 / singleVisThreshold;

			colorStateMap.at<cv::Vec3b>(i, j)[0] = 255 - peopleNum;//总人流量的图为黑色
			colorStateMap.at<cv::Vec3b>(i, j)[1] = 255 - peopleNum;
			colorStateMap.at<cv::Vec3b>(i, j)[2] = 255 - peopleNum;

			toRight.at<cv::Vec3b>(i, j)[0] = 255;//向右的图为蓝色
			toRight.at<cv::Vec3b>(i, j)[1] = 255 - toRightPeopleNum;
			toRight.at<cv::Vec3b>(i, j)[2] = 255 - toRightPeopleNum;

			toUp.at<cv::Vec3b>(i, j)[0] = 255 - toUpPeopleNum;//向上的图为橙色
			toUp.at<cv::Vec3b>(i, j)[1] = (255 - 92)*(255 - toUpPeopleNum) / 255 + 92;
			toUp.at<cv::Vec3b>(i, j)[2] = 255;

			toLeft.at<cv::Vec3b>(i, j)[0] = 255 - toLeftPeopleNum;//向左的图为红色
			toLeft.at<cv::Vec3b>(i, j)[1] = 255 - toLeftPeopleNum;
			toLeft.at<cv::Vec3b>(i, j)[2] = 255;

			toDown.at<cv::Vec3b>(i, j)[0] = 255;//向下的图为紫色
			toDown.at<cv::Vec3b>(i, j)[1] = 255 - toDownPeopleNum;
			toDown.at<cv::Vec3b>(i, j)[2] = (255-158)*(255 - toDownPeopleNum)/255 + 158;

		}

	cv::Mat visualMap;
	visualMap = cv::Mat::zeros(StateMap.rows, StateMap.cols * 5, CV_8UC3);
	cv::Mat ROI[5];

	for (int i = 0; i < 5; i++)
		ROI[i] = visualMap(cv::Rect(i*StateMap.rows, 0, StateMap.cols, StateMap.rows));

	double x0 = 50;//箭头的中点
	double y0 = 50;
	double arrowLenth = 50;

	cv::arrowedLine(toRight, cv::Point(x0 - arrowLenth / 2, y0), cv::Point(x0 + arrowLenth / 2, y0), cv::Scalar(255, 0, 0), 2);
	cv::arrowedLine(toLeft, cv::Point(x0 + arrowLenth / 2, y0), cv::Point(x0 - arrowLenth / 2, y0), cv::Scalar(0, 0, 255), 2);
	cv::arrowedLine(toDown, cv::Point(x0, y0 - arrowLenth / 2), cv::Point(x0, y0 + arrowLenth / 2), cv::Scalar(255, 0, 158), 2);
	cv::arrowedLine(toUp, cv::Point(x0, y0 + arrowLenth / 2), cv::Point(x0, y0 - arrowLenth / 2), cv::Scalar(0, 92, 255), 2);

	toRight.copyTo(ROI[4]);
	toLeft.copyTo(ROI[3]);
	colorStateMap.copyTo(ROI[2]);
	toDown.copyTo(ROI[1]);
	toUp.copyTo(ROI[0]);

	int lineWidth = 2;
	cv::Scalar color(0);
	cv::line(visualMap, cv::Point(StateMap.cols, 0), cv::Point(StateMap.cols, StateMap.rows), color, lineWidth);
	cv::line(visualMap, cv::Point(StateMap.cols * 2, 0), cv::Point(StateMap.cols * 2, StateMap.rows), color, lineWidth);
	cv::line(visualMap, cv::Point(StateMap.cols * 3, 0), cv::Point(StateMap.cols * 3, StateMap.rows), color, lineWidth);
	cv::line(visualMap, cv::Point(StateMap.cols * 4, 0), cv::Point(StateMap.cols * 4, StateMap.rows), color, lineWidth);
	cv::resize(visualMap, visualMap, cv::Size(1250, 250));

	cv::imwrite(outputDir + "_vis.png", visualMap);
	printf("Write state map vis to %s\n", outputDir + "_vis.png");

	//cv::Mat tmp(cv::Size(colorStateMap.cols, colorStateMap.rows), CV_8UC1);
	//channels[8].convertTo(tmp, CV_8UC1);
	//cv::applyColorMap(tmp, colorStateMap,cv::COLORMAP_BONE);
	//cv::cvtColor(colorStateMap, colorStateMap, cv::COLOR_HLS2BGR);
	//cv::imwrite(outputDir + "_color.jpg", colorStateMap);

	//添加png的alpha通道
	//std::vector<cv::Mat> png;
	//cv::split(colorStateMap, png);
	//png.push_back(alpha);
	//cv::merge(png, colorStateMap);

	//png.clear();
	//cv::split(toRight, png);
	//png.push_back(alpha);
	//cv::merge(png, toRight);

	//png.clear();
	//cv::split(toUp, png);
	//png.push_back(alpha);
	//cv::merge(png, toUp);

	//png.clear();
	//cv::split(toLeft, png);
	//png.push_back(alpha);
	//cv::merge(png, toLeft);

	//png.clear();
	//cv::split(toDown, png);
	//png.push_back(alpha);
	//cv::merge(png, toDown);

	//cv::imwrite(outputDir + "_color.png", colorStateMap);//输出带有alpha通道的png图像，这样可以把stateMap投到卫星图上
	//cv::imwrite(outputDir + "toLeft.png", toLeft);
	//cv::imwrite(outputDir + "toRight.png", toRight);
	//cv::imwrite(outputDir + "toUp.png", toUp);
	//cv::imwrite(outputDir + "toDown.png", toDown);

}


PedestrianSimulator::~PedestrianSimulator()
{
}