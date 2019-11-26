#include "PedestrianSimulator.h"



PedestrianSimulator::PedestrianSimulator(std::string sceneFile, std::string pedestrianFile, int VisualRate, std::string videoFile,std::string outputFile)
{

	visualRate = VisualRate;
	sceneStructure.InitScene(sceneFile);
	cv::FileStorage storage(pedestrianFile, cv::FileStorage::READ);
	storage["matrix"] >> pedestrianMatrix;//��ȡ�����ṹ
	for (int i = 0; i < pedestrianMatrix.rows; i++)
	{
		for (int j = 0; j < pedestrianMatrix.cols; j++) {
			std::cout << pedestrianMatrix.at<double>(i, j) << " ";//��ȡ�����ܶȾ���
		}
		std::cout << std::endl;
	}
	
	if(videoFile!="")
		if (!video.open(videoFile, CV_FOURCC('X', 'V', 'I', 'D'), 20, cv::Size(500, 500))) {
			std::cout << "Cannot open video file " << videoFile << ", program exit.\n";
			exit(-4);
		}

	tmpMatrix = cv::Mat::zeros(cv::Size(pedestrianMatrix.cols, pedestrianMatrix.cols), CV_32F);
	StateMap = cv::Mat::zeros(cv::Size(sceneStructure.sceneState.rows, sceneStructure.sceneState.cols), CV_32FC(9));//��ʼ��״̬��ͼ
	outputDir = outputFile;
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
			if (prob*10 < uniform(gen)) continue;//����һ��������������

			tmpMatrix.at<float>(i, j)++;

			double tmp1, tmp2, dis1,dis2;
			do {
				PedestrianInfo onePedestrian(sceneStructure.StartEndPositions[i], sceneStructure.StartEndPositions[j]);
				dis1 = sceneStructure.GetClosestObstacle(onePedestrian.curPosition.x, onePedestrian.curPosition.y, tmp1, tmp2);//���˲��ܳ������ϰ�����
				dis2 = sceneStructure.GetClosestObstacle(onePedestrian.tarPostion.x, onePedestrian.tarPostion.y, tmp1, tmp2);//Ŀ�ĵز��������ϰ�����
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

	for (int i = 0; i < pedestrians.size(); i++) {//�ж�������û�е����յ㣬��������ɾ��
		double dis1 = (pedestrians[i].curPosition.x - pedestrians[i].tarPostion.x)*(pedestrians[i].curPosition.x - pedestrians[i].tarPostion.x)
			+ (pedestrians[i].curPosition.y - pedestrians[i].tarPostion.y)*(pedestrians[i].curPosition.y - pedestrians[i].tarPostion.y);
		dis1 = sqrt(dis1);
		double tmp1, tmp2;
		double dis2 = sceneStructure.GetClosestObstacle(pedestrians[i].curPosition.x, pedestrians[i].curPosition.y, tmp1, tmp2);
		if (dis1 < 1 || fabs(dis2) < 0.1)//����������������ߵ��ϰ������ˣ��Ǿ�ɾ������
		//if(dis1 < 1)
		{
			pedestrians.erase(pedestrians.begin() + i);
			i--;
		}
		
	}

	//�������˵�λ�ú��ٶ�
	for (auto &p : pedestrians) {
		p.lastPosition.x = p.curPosition.x;
		p.lastPosition.y = p.curPosition.y;

		double curSpeed = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));
		if (curSpeed / p.tarSpeed > 1.3) {//�����ٶ�����ΪĿ���ٶȵ�1.3��
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

	//�������˵ļ��ٶ�
	for (auto &p : pedestrians) {
		Speed targetSpeed;
		targetSpeed.vx = p.tarPostion.x - p.curPosition.x;
		targetSpeed.vy = p.tarPostion.y - p.curPosition.y;
		double lenth = sqrt(targetSpeed.vx*targetSpeed.vx + targetSpeed.vy*targetSpeed.vy);
		targetSpeed.vx /= lenth;
		targetSpeed.vy /= lenth;
		targetSpeed.vx *= p.tarSpeed;
		targetSpeed.vy *= p.tarSpeed;

		Acceleration a1;//���ٶ��еĵ�һ���Ŀ�ĵ���ɵļ��ٶ�
		a1.ax = (targetSpeed.vx - p.curSpeed.vx) / 1;
		a1.ay = (targetSpeed.vy - p.curSpeed.vy) / 1;



		Acceleration a2;//���ٶȵڶ������Χ���˶Ե�ǰ���˵ĳ���
		a2.ax = 0;
		a2.ay = 0;
		double Vab0 = 2.8;//���������������2.1
		double sigma = 0.5;//���������������0.3
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
			double cosTheta = p.curSpeed.vx * (q.curPosition.x - p.curPosition.x) + p.curSpeed.vy *(q.curPosition.y - p.curPosition.y);//��������ǲ������ҵ���Ұ��Χ��
			cosTheta /= (sqrt(p.curSpeed.vx*p.curSpeed.vx + p.curSpeed.vy*p.curSpeed.vy) * disTwoPerson);
			double Theta = acos(cosTheta) * 180 / CV_PI;
			if (Theta > 100)//��������Ұ��Χ��100*2��֮����ˣ��Ӷ�����
				coefficient = 0.2;

			a2.ax += dir_x * Vab * coefficient;
			a2.ay += dir_y * Vab * coefficient;

		}


		Acceleration a3;//���ٶȵ������ǽ�ڶԵ�ǰ���˵ĳ���
		a3.ax = 0;
		a3.ay = 0;
		double UaB0 = 10;//���������������10m
		double R = 0.6;//���������������0.2m����֮ǰ�õ���0.6
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

		/*���ڿ���ǽ�ڣ���Ŀ�ĵ�λ��ǽ��ͬһ��ʱ�������ٶȻ����Ƚ��ͣ���Ϊǽ�ĳ���������Ŀ�ĵض��˵Ĵ󲿷�����������������������Ӧ���޸ģ�������⵽�����ٶȺܵͣ���Ŀ�ĵ�������ǽ�ڳ�������ʱ���ͽ�Ŀ�ĵض��˵�������ȫ�ӵ�����Ŀǰ��ǰ�������ϣ������ͱ�֤�˼�ʹ�˿���ǽ�ڣ�Ҳ�������ԭ���㷨�еĴ���ȼ������⡣*/
		double curSpeed = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));//��ǰ���ٶȴ�С������
		double curAccPersonAndObst = sqrt(pow(a1.ax + a3.ax, 2) + pow(a1.ay + a3.ay, 2));//��ǰĿ�ĵ�������ǽ���������͵Ĵ�С������
		double curAccPerson = sqrt(pow(a1.ax, 2) + pow(a1.ay, 2));//��ǰĿ�ĵض����˵�������С������

		if (curSpeed < p.tarSpeed*2 &&   curAccPersonAndObst< curAccPerson*0.8) {//��������ٶȵͣ���ǽ�ڳ��������˴󲿷�Ŀ�ĵ�����
			double lenth = sqrt(pow(p.curSpeed.vx, 2) + pow(p.curSpeed.vy, 2));//
			double curDirx = p.curSpeed.vx / lenth;//��ǰǰ������
			double curDiry = p.curSpeed.vy / lenth;//��ǰǰ������ע���ǵ�λ����
			double horizonAcc = a1.ax*curDirx + a1.ay*curDiry;//��ǰĿ�ĵ�����������ǰ�������ϵķ���
			a1.ax -= horizonAcc   * curDirx;//
			a1.ay -= horizonAcc   * curDiry;//�ȼ���Ŀ�ĵ�����ˮƽ�������������Ŀ�ĵ�����ֻ�д�ֱ��ǽ�ڵķ�����
			a1.ax += curAccPerson * curDirx;
			a1.ay += curAccPerson * curDiry;//���ڣ�ƽ����ǽ�ڵ����Ĵ�Сֱ�ӵ���Ŀ�ĵ�������С��
			//a3.ax = a3.ay = 0;//���Ӳ�Ҫǽ�ڵĳ����ˣ�
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

		if (sceneStructure.CoordinateConventer(cx, cy, ix, iy)) {
			StateMap.at<cv::Vec<float, 9>>(cv::Point(ix, iy))[dir] ++;
			StateMap.at<cv::Vec<float, 9>>(cv::Point(ix, iy))[8] ++;
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
	storage.release();
	printf("Write state map to %s\n", outputDir + ".yml");



	double maxValue, minValue;
	cv::minMaxIdx(channels[8], &minValue, &maxValue);
	for (int i = 0; i < StateMap.rows; i++)
		for (int j = 0; j < StateMap.cols; j++) {
			channels[8].at<float>(i, j) = channels[8].at<float>(i, j) / maxValue;
			channels[8].at<float>(i, j) = (1 - channels[8].at<float>(i, j))*255;
		}
	//cv::imshow("test", channels[8]);
	//cv::waitKey(0);
	cv::imwrite(outputDir + ".jpg", channels[8]);
	printf("Write state map to %s\n", outputDir + ".jpg");
	

}


PedestrianSimulator::~PedestrianSimulator()
{
}