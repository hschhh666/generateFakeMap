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

	std::normal_distribution<> tarv{ 1.34,0.26 };//��������Ǵ������ﳭ��
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
		return 0;//�����ǰλ���Ѿ������˵�ͼ��Χ������Ϊ�Ѿ����ϰ�������

	double mindis = 99999999;
	double range = 10;//meter�����10�׷�Χ�ڶ�û���ϰ������Ϊ������ϰ�����99999999��֮��
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
	if (mindis < pixelSize)//��������ϰ���ľ���ȵ�ͼ�ֱ��ʻ�С������Ϊ�Ѿ����ϰ�������
		mindis = 0;

	return mindis;
}

//ȫ��������ͼ�������ת�����������ȫ�����������ڸõ�ͼ���򷵻�true�����򷵻�false
//ȫ�ֵ�ͼ�У��������Ϊx������Ϊ��������Ϊy������Ϊ������ȡ����������ϵ�����Ҷ����ϱ�
//imgxָ�������꣬����Ϊ����imgyָ�������꣬����Ϊ����һ��Ҫע�����������ϵ�Ķ��塣��Ϊ��opencv�У�cv::Point(x,y)��x��y��������ǰ�涨�����ͬ��
bool SceneStructure::CoordinateConventer(double x, double y, int & imgx, int & imgy)
{
	imgx = (x - leftUpCornerX) / pixelSize;
	imgy = (leftUpCornerY - y) / pixelSize;
	if ((imgx >= imgSize) || (imgy >= imgSize) || (imgx < 0) || (imgy < 0))
		return false;
	return true;
}

//ȫ��������ͼ�������ת�����������ͼ�������ڸ�ͼ��Χ���򷵻�true�����򷵻�false��
//imgx��imgyָͼ������ϵ��ͼ���е�����ϵ��x��yָȫ�ֵ�ͼ������ϵ
//imgxָ�������꣬����Ϊ����imgyָ�������꣬����Ϊ����һ��Ҫע�����������ϵ�Ķ��塣��Ϊ��opencv�У�cv::Point(x,y)��x��y��������ǰ�涨�����ͬ��
//ͼ����ĵ�ͼ�У��������Ϊx������Ϊ��������Ϊy������Ϊ������ȡ����������ϵ�����Ҷ����ϱ�
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
