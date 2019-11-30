#include "PedestrianSimulator.h"



PedestrianSimulator::PedestrianSimulator(std::string sceneFile, std::string pedestrianFile, int VisualRate, std::string videoFile,std::string outputFile)
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

	tmpMatrix = cv::Mat::zeros(cv::Size(pedestrianMatrix.cols, pedestrianMatrix.cols), CV_32F);
	StateMap = cv::Mat::zeros(cv::Size(sceneStructure.sceneState.rows, sceneStructure.sceneState.cols), CV_32FC(9));//初始化状态地图
	outputDir = outputFile;
	
	double kernelSize = 1;//计算以人为中心，kernelSize米范围内
	double pixelSize = sceneStructure.pixelSize;
	int kernelPixelSize = 2*(kernelSize / pixelSize) + 1;
	double theta = 0.4;//高斯核的衰减半径，theta时衰减到0.6
	
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

			double tmp1, tmp2, dis1,dis2;
			do {
				PedestrianInfo onePedestrian(sceneStructure.StartEndPositions[i], sceneStructure.StartEndPositions[j]);
				dis1 = sceneStructure.GetClosestObstacle(onePedestrian.curPosition.x, onePedestrian.curPosition.y, tmp1, tmp2);//行人不能出生在障碍物上
				dis2 = sceneStructure.GetClosestObstacle(onePedestrian.tarPostion.x, onePedestrian.tarPostion.y, tmp1, tmp2);//目的地不可以在障碍物上
				
				if(dis1>=0.1 && dis2 >=0.1)
					pedestrians.push_back(onePedestrian);

			} while (dis1<0.1 || dis2<0.1);



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
		dis1 = sqrt(dis1);
		double tmp1, tmp2;
		double dis2 = sceneStructure.GetClosestObstacle(pedestrians[i].curPosition.x, pedestrians[i].curPosition.y, tmp1, tmp2);
		if (dis1 < 1 || fabs(dis2) < 0.1)//如果行人走着走着走到障碍物上了，那就删掉他！
		//if(dis1 < 1)
		{
			pedestrians.erase(pedestrians.begin() + i);
			i--;
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
		double Vab0 = 2.8;//这个参数论文里是2.1
		double sigma = 0.5;//这个参数论文里是0.3
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

		if (curSpeed < p.tarSpeed*2 &&   curAccPersonAndObst< curAccPerson*0.8) {//如果行人速度低，且墙壁斥力抵消了大部分目的地引力
			double lenth = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));//
			double curDirx = p.curSpeed.vx / lenth;//当前前进方向
			double curDiry = p.curSpeed.vy / lenth;//当前前进方向，注意是单位向量
			double horizonAcc = a1.ax*curDirx + a1.ay*curDiry;//当前目的地引力在行人前进方向上的分量
			a1.ax -= horizonAcc   * curDirx;//
			a1.ay -= horizonAcc   * curDiry;//先减掉目的地引力水平方向分量，现在目的地引力只有垂直于墙壁的分量了
			a1.ax += curAccPerson * curDirx;
			a1.ay += curAccPerson * curDiry;//现在，平行于墙壁的力的大小直接等于目的地引力大小了
			//a3.ax = a3.ay = 0;//老子不要墙壁的斥力了！
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
		
		//为了省事，stateMap的channel 0 用来记做向右走的人，channel 1 记向左走的人， channel 8 记总人数。即channel 0 + channel 1 = channel8。也就是说，剩下的channel暂时没用
		if (p.tarPostion.x > p.initPosition.x)//向右走
			dir = 0;
		else
			dir = 1;

		int kernelPixelSize = stateMapKernal.cols;
		if (sceneStructure.CoordinateConventer(cx, cy, ix, iy)) {

			for (int i = 0;i<kernelPixelSize;i++)
				for (int j = 0; j < kernelPixelSize; j++) {
					float value = stateMapKernal.at<float>(i, j);
					int tmp_y = i - (kernelPixelSize / 2);
					int tmp_x = j - (kernelPixelSize / 2);
					if (ix + tmp_x >= sceneStructure.imgSize || ix + tmp_x < 0 || iy + tmp_y >= sceneStructure.imgSize || iy + tmp_y < 0)
						continue;
					StateMap.at<cv::Vec<float, 9>>(cv::Point(ix + tmp_x, iy + tmp_y))[8] += value;//以人为中心，做高斯衰减
					StateMap.at<cv::Vec<float, 9>>(cv::Point(ix + tmp_x, iy + tmp_y))[dir] += value;
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
	storage << "toLeft"   << channels[1];

	storage.release();
	printf("Write state map to %s\n", outputDir + ".yml");

	double maxL3 = 0;

	cv::Mat colorStateMap = cv::Mat::zeros(StateMap.rows, StateMap.cols, CV_8UC3);//该mat保存的是彩色状态图
	for (int i = 0;i<StateMap.rows;i++)
		for (int j = 0; j < StateMap.cols; j++) {
			float L1 = StateMap.at<cv::Vec<float, 9>>(i, j)[0];//该栅格中向右走的人数
			float L2 =  StateMap.at<cv::Vec<float, 9>>(i, j)[1];//该栅格中向左走的人数
			float theta1 = 0;//向右走为红色
			float theta2  = (170/255.0)*180.0;//向左走为蓝色。在windows定义的HSL色彩模式中，h=170是蓝色。windows定义的hsl中h的范围是[0,255]，opencv定义的hls中h的范围是[0,180]，这里做了一个从windows到opencv色彩定义的转换
			theta1 = theta1 * CV_PI / 180;
			theta2 = theta2 * CV_PI / 180;
			float L3 = sqrt(L1 * L1 + L2 * L2 + 2 * L1 * L2 * cos(theta1 - theta2));//这一步计算的是中间变量
			if (L3 < 1e-3) {
				colorStateMap.at<cv::Vec3b>(i, j)[0] = 0;
				colorStateMap.at<cv::Vec3b>(i, j)[1] = 255;
				colorStateMap.at<cv::Vec3b>(i, j)[2] = 0;
				continue;
			}
			float theta3 = acos((L1*cos(theta1) + L2 * cos(theta2)) / L3);//使用opencv的hls颜色模式为行人方向赋值，其中令饱和度都为255。定义向右走为红色，向左走为蓝色，颜色的亮度由人数决定，人越多亮度越接近128，人越少亮度越接近255。人流交汇处的颜色由红色/蓝色混合而成，其色调（向量方向）等于两股人流对应颜色的向量和的方向，亮度等于两股人流的亮度之和
			theta3 = theta3 * 180 / CV_PI;//计算该栅格的颜色，对应opencv中hls颜色模式的h（色调）
			L3 = L1 + L2;//计算该栅格的亮度，对应opencv中hls颜色模式的l（亮度）。人越多亮度越接近128，人越少亮度越接近255
			
			if (L3 > maxL3)
				maxL3 = L3;

			L3 = L3 > 128 ? 128 : L3;//亮度为255时对应白色了，亮度为0时对应黑色了，亮度为128时是由白变黑的中间点
			colorStateMap.at<cv::Vec3b>(i, j)[0] = theta3;
			colorStateMap.at<cv::Vec3b>(i, j)[1] = 255 - L3;
			colorStateMap.at<cv::Vec3b>(i, j)[2] = 255;//饱和度都为255
		}
	printf("Max people = %.2lf\n", maxL3);
	
	cv::cvtColor(colorStateMap, colorStateMap, cv::COLOR_HLS2BGR);
	cv::imwrite(outputDir + "_color.jpg", colorStateMap);

	double maxValue, minValue;
	cv::minMaxIdx(channels[8], &minValue, &maxValue);
	for (int i = 0; i < StateMap.rows; i++)
		for (int j = 0; j < StateMap.cols; j++) {
			//channels[8].at<float>(i, j) = channels[8].at<float>(i, j) / maxValue;
			//channels[8].at<float>(i, j) = (1 - channels[8].at<float>(i, j))*255;
			if (channels[8].at<float>(i, j) > 255)
				channels[8].at<float>(i, j) = 255;
			channels[8].at<float>(i, j) = 255 - channels[8].at<float>(i, j);

		}
	//cv::imshow("test", channels[8]);
	//cv::waitKey(0);
	cv::imwrite(outputDir + ".jpg", channels[8]);
	printf("Write state map to %s\n", outputDir + ".jpg");
	

}


PedestrianSimulator::~PedestrianSimulator()
{
}