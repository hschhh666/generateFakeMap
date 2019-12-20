#include "DataManager.h"

PedestrianInfo::PedestrianInfo(Position cur, Position tar)
{
	static std::random_device rd{};
	static std::mt19937 gen{ rd() };

	std::normal_distribution<> px{ cur.x,sigma_px };
	initPosition.x = lastPosition.x =  curPosition.x = px(gen);
	std::normal_distribution<> py{ cur.y,sigma_py };
	initPosition.y = lastPosition.y =  curPosition.y = py(gen);


	std::normal_distribution<> tx{ tar.x,sigma_px };
	tarPostion.x = tx(gen);
	std::normal_distribution<> ty{ tar.y,sigma_py };
	tarPostion.y = ty(gen);

	std::normal_distribution<> tarv{ 1.34,0.26 };//这个参数是从论文里抄的
	//std::normal_distribution<> tarv{ 2.68,0.26 };
	tarSpeed = tarv(gen);

	curSpeed.vx = tarPostion.x - curPosition.x;
	curSpeed.vy = tarPostion.y - curPosition.y;
	double lenth = sqrt(curSpeed.vx*curSpeed.vx + curSpeed.vy*curSpeed.vy);
	curSpeed.vx /= lenth;
	curSpeed.vy /= lenth;
	curSpeed.vx *= tarSpeed;
	curSpeed.vy *= tarSpeed;

	curAcc.ax = 0;
	curAcc.ay = 0;

}

PedestrianInfo::PedestrianInfo(std::vector<Position> roadPoints_)
{
	Position cur;
	Position tar;
	cur.x = roadPoints_.at(0).x;//起始位置就是第一个点
	cur.y = roadPoints_.at(0).y;
	tar.x = roadPoints_.at(1).x;//最初的目标位置是第二个点。当然，如果总共就只有俩点的话那这个点也就是最终的点了。
	tar.y = roadPoints_.at(1).y;

	endPosition.x = endPosition.y = -10;//如果endPosition为负数的话，就表明只有起点和终点俩点，没有中间点

	static std::random_device rd{};
	static std::mt19937 gen{ rd() };

	std::normal_distribution<> px{ cur.x,sigma_px };
	initPosition.x = lastPosition.x = curPosition.x = px(gen);
	std::normal_distribution<> py{ cur.y,sigma_py };
	initPosition.y = lastPosition.y = curPosition.y = py(gen);


	std::normal_distribution<> tx{ tar.x,sigma_px };
	tarPostion.x = tx(gen);
	std::normal_distribution<> ty{ tar.y,sigma_py };
	tarPostion.y = ty(gen);

	std::normal_distribution<> tarv{ 1.34,0.26 };//这个参数是从论文里抄的
	//std::normal_distribution<> tarv{ 2.68,0.26 };
	tarSpeed = tarv(gen);

	curSpeed.vx = tarPostion.x - curPosition.x;
	curSpeed.vy = tarPostion.y - curPosition.y;
	double lenth = sqrt(curSpeed.vx*curSpeed.vx + curSpeed.vy*curSpeed.vy);
	curSpeed.vx /= lenth;
	curSpeed.vy /= lenth;
	curSpeed.vx *= tarSpeed;
	curSpeed.vy *= tarSpeed;

	curAcc.ax = 0;
	curAcc.ay = 0;

	if (roadPoints_.size() > 2) {
		endPosition.x = roadPoints_.at(2).x;//如果有仨点的话，最终最终目的地是第三个点
		endPosition.y = roadPoints_.at(2).y;


		//std::normal_distribution<> ex{endPosition.x,0 };
		//std::normal_distribution<> ey{endPosition.y,0 };
		//endPosition.x = ex(gen);
		//endPosition.y = ey(gen);

	}

}



void SceneStructure::InitScene(std::string filename, std::string satelliteFile)
{
	leftUpCornerX = leftUpCornerY = 0;
	cv::FileStorage storage(filename, cv::FileStorage::READ);
	if (!storage.isOpened()) {
		std::cout << "Cannot open file " << filename << " ,press any key to exit.\n";
		getchar();
		exit(-1);
	}

	storage["sceneSize"] >> sceneSize;
	storage["pixelSize"] >> pixelSize;
	imgSize = sceneSize / pixelSize;
	int targetPositionsNum;
	storage["targetPositionsNum"] >> targetPositionsNum;
	for (int i = 0; i < targetPositionsNum; i++) {
		int x, y;
		storage["targetPosition" + std::to_string(i) + "x"] >> x;
		storage["targetPosition" + std::to_string(i) + "y"] >> y;
		Position p;
		p.x = x;
		p.y = y;
		StartEndPositions.push_back(p);
	}

	int hiddenPositionsNum;
	storage["hiddenPositionsNum"] >> hiddenPositionsNum;
	for (int i = 0; i < hiddenPositionsNum; i++) {
		int x, y;
		storage["hiddenPosition" + std::to_string(i) + "x"] >> x;
		storage["hiddenPosition" + std::to_string(i) + "y"] >> y;
		Position p;
		p.x = x;
		p.y = y;
		HiddenPositions.push_back(p);
	}

	storage["HiddenMatrix"] >> HiddenMatrix;
	storage["structure"] >> scene;
	//for (int i = 0; i < HiddenMatrix.cols; i++)
	//{
	//	for (int j = 0; j < HiddenMatrix.cols; j++)
	//		std::cout << HiddenMatrix.at<double>(i, j) << " ";
	//	std::cout << std::endl;
	//}	

	cv::Mat satelliteMap = cv::imread(satelliteFile);
	if (satelliteMap.empty()) {
		std::cout << "Cannot open file " << satelliteFile << " ,press any key to exit.\n";
		getchar();
		exit(-1);
	}

	if (satelliteMap.cols != int(sceneSize / pixelSize)) {
		printf("Warning! Satellite map size is %d x %d pixels\n" , satelliteMap.cols, satelliteMap.cols);
	}

	cv::resize(satelliteMap, satelliteMap, cv::Size(int(sceneSize / pixelSize), int(sceneSize / pixelSize)));

	sceneState = scene.clone();//之前是显示场景的mask，现在改成显示场景的卫星图
	sceneState = satelliteMap.clone();


	for (int i = 0; i < targetPositionsNum; i++)
	{
		cv::circle(sceneState, cv::Point(StartEndPositions[i].x, StartEndPositions[i].y), 4, cv::Scalar(0, 0, 255), -1);
		cv::putText(sceneState, std::to_string(i), cv::Point(StartEndPositions[i].x + 5, StartEndPositions[i].y + 5), 1, 1, cv::Scalar(0, 0, 0));
	}

	//for (int i = 0; i < hiddenPositionsNum; i++)
	//{
	//	cv::circle(sceneState, cv::Point(HiddenPositions[i].x, HiddenPositions[i].y), 4, cv::Scalar(0, 255, 0), -1);
	//	cv::putText(sceneState, std::to_string(i), cv::Point(HiddenPositions[i].x + 5, HiddenPositions[i].y + 5), 1, 1, cv::Scalar(0, 0, 0));
	//}

	
	for (auto &p : StartEndPositions) {//把各个拓扑点的坐标由图像坐标转换为全局坐标
		int x = p.x;
		int y = p.y;
		CoordinateConventer(x, y, p.x, p.y);
	}

	for (auto &p : HiddenPositions) {//把各个拓扑点的坐标由图像坐标转换为全局坐标
		int x = p.x;
		int y = p.y;
		CoordinateConventer(x, y, p.x, p.y);
	}

	imgSize = sceneSize / pixelSize;

}


double SceneStructure::GetClosestObstacle(double px, double py, double & ox, double & oy)// px: position x     ;     ox: obstacle x
{
	int ix,iy;//image x
	int x, y;

	CoordinateConventer(px, py, ix, iy);

	if (abs(ix) >= imgSize || abs(iy) >= imgSize)
		return 0;//如果当前位置已经超出了地图范围，则认为已经在障碍物上了

	double mindis = 99999999;
	double range = 10;//meter，如果10米范围内都没有障碍物，则认为最近的障碍物在99999999米之外
	int imageRange = range / pixelSize;
	for(x = ix - imageRange;x<=ix+imageRange;x++)
		for (y = iy - imageRange; y <= iy + imageRange; y++) {
			if (x < 0 || x >= imgSize || y < 0 || y >= imgSize)continue;
			if (scene.at<cv::Vec3b>(cv::Point(x,y))[0] != 0) continue;

			double tmpx, tmpy;
			CoordinateConventer(x, y, tmpx, tmpy);
			double dis = (tmpx - px)*(tmpx - px) + (tmpy - py)* (tmpy - py);
			dis = sqrt(dis);

			if (dis <= mindis) {
				mindis = dis;
				ox = tmpx;
				oy = tmpy;
			}
		}
	if (mindis < pixelSize)//如果距离障碍物的距离比地图分辨率还小，则认为已经在障碍物上了
		mindis = 0;

	return mindis;
}

//全局坐标与图像坐标的转换，若输入的全局坐标点出现在该地图中则返回true，否则返回false
//全局地图中，定义横向为x，向右为正；纵向为y，向上为正。采取东北天坐标系，即右东，上北
//imgx指横向坐标，向右为正；imgy指纵向坐标，向下为正。一定要注意这里对坐标系的定义。因为在opencv中，cv::Point(x,y)的x，y坐标与我前面定义的相同。
bool SceneStructure::CoordinateConventer(double x, double y, int & imgx, int & imgy)
{
	imgx = (x - leftUpCornerX) / pixelSize;
	imgy = (leftUpCornerY - y) / pixelSize;
	if ((imgx >= imgSize) || (imgy >= imgSize) || (imgx < 0) || (imgy < 0))
		return false;
	return true;
}

//全局坐标与图像坐标的转换，若输入的图像坐标在该图像范围内则返回true，否则返回false；
//imgx和imgy指图像坐标系，图像中的坐标系；x，y指全局地图的坐标系
//imgx指横向坐标，向右为正；imgy指纵向坐标，向下为正。一定要注意这里对坐标系的定义。因为在opencv中，cv::Point(x,y)的x，y坐标与我前面定义的相同。
//图像定义的地图中，定义横向为x，向右为正；纵向为y，向上为正。采取东北天坐标系，即右东，上北
bool SceneStructure::CoordinateConventer(int imgx, int imgy, double &x, double &y)
{
	x = ((double)imgx)*pixelSize + leftUpCornerX;
	y = leftUpCornerY - ((double)imgy)*pixelSize;

	if (imgx < 0 || imgy < 0 || imgx >= imgSize || imgy >= imgSize)
		return false;
	return true;
}


void SceneStructure::ShowScene(int key, std::string imgname)
{	
	cv::imshow(imgname, sceneState);
	cv::waitKey(key);
	
}
