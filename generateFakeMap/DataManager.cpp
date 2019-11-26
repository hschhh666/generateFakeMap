#include "DataManager.h"

PedestrianInfo::PedestrianInfo(Position cur, Position tar)
{
	static std::random_device rd{};
	static std::mt19937 gen{ rd() };

	std::normal_distribution<> px{ cur.x,sigma_px };
	lastPosition.x =  curPosition.x = px(gen);
	std::normal_distribution<> py{ cur.y,sigma_py };
	lastPosition.y =  curPosition.y = py(gen);


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



void SceneStructure::InitScene(std::string filename)
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
	storage["structure"] >> scene;
	sceneState = scene.clone();
	for (int i = 0; i < targetPositionsNum; i++)
	{
		cv::circle(sceneState, cv::Point(StartEndPositions[i].x, StartEndPositions[i].y), 4, cv::Scalar(0, 0, 255), -1);
		cv::putText(sceneState, std::to_string(i), cv::Point(StartEndPositions[i].x + 5, StartEndPositions[i].y + 5), 1, 1, cv::Scalar(0, 0, 0));
	}
	
	for (auto &p : StartEndPositions) {
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
