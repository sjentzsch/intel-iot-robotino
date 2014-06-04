/*
 * LaserScanner.cpp
 *
 *  Created on: Mar 30, 2014
 *      Author: root
 */

#include "LaserScanner.h"
#include "../SensorServer.h"
#include <cmath>
#include "BaseParameterProvider.h"
#include "LaserScannerApi.h"

LaserScanner::LaserScanner(SensorServer* sensorServer): execThread(NULL), signal(LaserScannerSignal::RUN)
{

	// setting up physical laser calibration
	float offset_x = 0.218;
	float offset_y = 0.0;
	float theta = 0.0;

	T_laser2base[0] = cos(theta);  	T_laser2base[1] = -sin(theta);		T_laser2base[2] = offset_x;
	T_laser2base[3] = sin(theta);	T_laser2base[4] = cos(theta);		T_laser2base[5] = offset_y;
	T_laser2base[6] = 0.0;			T_laser2base[7] = 0.0;				T_laser2base[8] = 1.0;

	scannerDriver = new LaserScannerApi();

	this->sensorServer = sensorServer;




	WorldModel* worldModel = ModelProvider::getInstance()->getWorldModel();

	const int WALL_THICKNESS = 20;	// in cm
	const int MACHINE_RADIUS = 25;	// in cm

	//TODO FIXME map size use define or config file!!

	laserscannerMap.create(560, 1120, CV_8UC1);
	laserscannerMap = Scalar(255);

	// Walls
	rectangle(laserscannerMap, Point(0,0), Point(WALL_THICKNESS,560), Scalar(0), CV_FILLED, CV_AA, 0);
	rectangle(laserscannerMap, Point(0,0), Point(1120,WALL_THICKNESS), Scalar(0), CV_FILLED, CV_AA, 0);
	rectangle(laserscannerMap, Point(1120-WALL_THICKNESS,0), Point(1120,560), Scalar(0), CV_FILLED, CV_AA, 0);
	rectangle(laserscannerMap, Point(0,560-WALL_THICKNESS), Point(1120,560), Scalar(0), CV_FILLED, CV_AA, 0);

	// Production Machines + Recycling Machine (LEFT + RIGHT AREA)
	for(unsigned int i=0; i<13; i++)
	{
		float currY = (worldModel->poi[i].y+1)*Y_GRID_WIDTH;
		float currX = (worldModel->poi[i].x+1)*X_GRID_WIDTH;
		circle(laserscannerMap, Point(currY/10, currX/10), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);
	}
	for(unsigned int i=16; i<29; i++)
	{
		float currY = (worldModel->poi[i].y+1)*Y_GRID_WIDTH;
		float currX = (worldModel->poi[i].x+1)*X_GRID_WIDTH;
		circle(laserscannerMap, Point(currY/10, currX/10), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);
	}

	// LEFT AREA: delivery gates
	circle(laserscannerMap, Point(20, 245), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);
	circle(laserscannerMap, Point(20, 280), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);
	circle(laserscannerMap, Point(20, 315), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);

	// RIGHT AREA: delivery gates
	circle(laserscannerMap, Point(1100, 245), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);
	circle(laserscannerMap, Point(1100, 280), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);
	circle(laserscannerMap, Point(1100, 315), MACHINE_RADIUS, Scalar(0), CV_FILLED, CV_AA, 0);

	//GaussianBlur(laserscannerMap, laserscannerMap, Size(31, 31), 0, 0);

	/*namedWindow("laserscannerMap", CV_WINDOW_AUTOSIZE);
	imshow("laserscannerMap", laserscannerMap);
	waitKey(0);*/
}

LaserScanner::~LaserScanner()
{
	delete scannerDriver;
}

void LaserScanner::startThread()
{
	execThread = new boost::thread(&LaserScanner::loop, this);
}

void LaserScanner::loop()
{
	while(true)
	{
		checkSignalStatus();

		/*if (puckDetection == CameraPuckDetection::OFF) {
			boost::mutex::scoped_lock l(m_mutex);
			puckState = CameraPuckState::OFF;
			vPucksFinal.clear();
			vPucksFinalProcessed = true;
		} else
			calcPuckDetection();*/

		LaserScannerReadings scan = this->readings();
		//cout << "*****************" << endl;
		/*cout << "angle_min: " << scan.api_readings.angle_min << endl;
		cout << "angle_max: " << scan.api_readings.angle_max << endl;
		cout << "angle_increment: " << scan.api_readings.angle_increment << endl;
		cout << "time_increment: " << scan.api_readings.time_increment << endl;
		cout << "scan_time: " << scan.api_readings.scan_time << endl;
		cout << "range_min: " << scan.api_readings.range_min << endl;
		cout << "range_max: " << scan.api_readings.range_max << endl;*/
		/*for(unsigned int i=0; i<scan.positions.size(); i+=50)
		{
			cout << i << ": " << scan.ranges[i] << endl;
		}
		if(scan.positions.size() > 0)
		{
			cout << "middle: " << scan.ranges[scan.positions.size()/2] << endl;
		}*/

		float my_x, my_y, my_phi;
		this->sensorServer->getOdometry(my_x, my_y, my_phi);
		std::vector<bool> isDynObstacle(scan.positions.size());
		std::vector<bool> isValid(scan.positions.size());
		int numDynObstacles = 0;
		for(unsigned int i=0; i<scan.positions.size(); i++)
		{
			this->sensorServer->transformBase2World(scan.positions.at(i)[0], scan.positions.at(i)[1], my_x, my_y, my_phi, scan.positionsGlob.at(i)[0], scan.positionsGlob.at(i)[1]);

			// if point is outside the field dimensions OR point range is outside the valid range of the laserscanner
			if(!scannerDriver->coordInsideField(scan.positionsGlob.at(i)[0], scan.positionsGlob.at(i)[1]) || !scan.inRange.at(i))
			{
				isValid.at(i) = false;
				isDynObstacle.at(i) = false;
			}
			else
			{
				isValid.at(i) = true;

				if((int)laserscannerMap.at<uchar>(Point(scan.positionsGlob.at(i)[1]*100,scan.positionsGlob.at(i)[0]*100)) < 255)
				{
					isDynObstacle.at(i) = false;
				}
				else
				{
					isDynObstacle.at(i) = true;
					numDynObstacles++;
				}
			}
		}
		//cout << "obstacle ratio: " << numDynObstacles << "/" << scan.positions.size() << endl;



		const unsigned int KERNEL_SUM_WIDTH = 13;	// should be odd
		const unsigned int CLUSTER_THRESHOLD_FRACTION = 2;
		const unsigned int KERNEL_MEDIAN_WIDTH = 13;	// should be odd
		const unsigned int CLUSTER_AVERAGE_DIFF_SIZE = 4;
		const float CLUSTER_MIN_DISTANCE = 0.4;
		const float OUTLIER_STD_MULTIPLIER = 1;
		const unsigned int MIN_POINTS_IN_CLUSTER = 8;

		unsigned int kernel_border_offset = (KERNEL_SUM_WIDTH-1)/2;
		std::vector<unsigned int> clusterSum(isDynObstacle.size());
		unsigned int prev_sum = 0;
		for(unsigned int i=0; i<kernel_border_offset && i<isDynObstacle.size(); i++)
			prev_sum += isDynObstacle.at(i);
		for(unsigned int i=0; i<isDynObstacle.size(); i++)
		{
			unsigned int border_left = kernel_border_offset+1 > i ? 0 : isDynObstacle.at(i-kernel_border_offset-1);
			unsigned int border_right = i+kernel_border_offset >= isDynObstacle.size() ? 0 : isDynObstacle.at(i+kernel_border_offset);
			prev_sum = clusterSum.at(i) = prev_sum - border_left + border_right;
		}

		unsigned int cluster_threshold = KERNEL_SUM_WIDTH/CLUSTER_THRESHOLD_FRACTION;
		std::vector<std::vector<unsigned int> > clusters;	// vector of clusters with indexStart and indexEnd
		bool currInsideCluster = false;
		for(unsigned int i=0; i<clusterSum.size(); i++)
		{
			if(!currInsideCluster && clusterSum.at(i) >= cluster_threshold)
			{
				std::vector<unsigned int> newCluster(2);
				newCluster.at(0) = i;
				clusters.push_back(newCluster);
				currInsideCluster = true;
			}
			else if(currInsideCluster && clusterSum.at(i) < cluster_threshold)
			{
				clusters.back().at(1) = i-1;
				currInsideCluster = false;
			}
		}
		if(currInsideCluster)
			clusters.back().at(1) = clusterSum.size()-1;

		/*for(unsigned int i=0; i<clusters.size(); i++)
			::std::cout << "Cluster " << i << ": " << clusters.at(i).at(0) << "," << clusters.at(i).at(1) << ::std::endl;*/

		kernel_border_offset = (KERNEL_MEDIAN_WIDTH-1)/2;
		unsigned int clusterCount = clusters.size();
		for(unsigned int c=0; c<clusterCount; c++)
		{
			std::vector<float> median(clusters.at(c).at(1)-clusters.at(c).at(0)+1);
			for(unsigned int i=clusters.at(c).at(0); i<=clusters.at(c).at(1); i++)
			{
				std::vector<float> currMedianContainer;
				for(unsigned int j=0; j<KERNEL_MEDIAN_WIDTH; j++)
				{
					unsigned int index = j+i-kernel_border_offset;
					if(j+i < kernel_border_offset || index >= scan.ranges.size())
						continue;
					if(!isDynObstacle.at(index))
						continue;
					currMedianContainer.push_back(scan.ranges.at(index));
				}

				::std::sort(currMedianContainer.begin(), currMedianContainer.end());
				median.at(i-clusters.at(c).at(0)) = currMedianContainer.size() == 0 ? 0 : currMedianContainer.at((currMedianContainer.size()-1)/2);
			}

			unsigned int currClusterIndex = c;
			unsigned int originalClusterBorder = clusters.at(c).at(1);
			for(unsigned int i=0; i<median.size()-1; i++)
			{
				float diff = abs(median.at(i+1) - median.at(i));

				if(diff > CLUSTER_MIN_DISTANCE)
				{
					clusters.at(currClusterIndex).at(1) = clusters.at(c).at(0)+i;
					std::vector<unsigned int> newCluster(2);
					newCluster.at(0) = clusters.at(c).at(0)+i+1;
					currClusterIndex = clusters.size();
					clusters.push_back(newCluster);
				}
			}
			clusters.at(currClusterIndex).at(1) = originalClusterBorder;
		}

		std::vector<std::vector<float> > clustersMid;
		for(unsigned int c=0; c<clusters.size(); c++)
		{
			std::vector<float> avg(2);
			float std;
			unsigned int counter = 0;
			for(unsigned int j=clusters.at(c).at(0); j<=clusters.at(c).at(1); j++)
			{
				if(!isDynObstacle.at(j))
					continue;

				avg.at(0) += scan.positionsGlob.at(j).at(0);
				avg.at(1) += scan.positionsGlob.at(j).at(1);
				counter++;
			}
			avg.at(0) /= counter;
			avg.at(1) /= counter;

			std::vector<float> dist(clusters.at(c).at(1)-clusters.at(c).at(0)+1);
			for(unsigned int j=clusters.at(c).at(0); j<=clusters.at(c).at(1); j++)
			{
				if(!isDynObstacle.at(j))
					continue;

				std += dist.at(j-clusters.at(c).at(0)) = SQUARE(avg.at(0)-scan.positionsGlob.at(j).at(0))+SQUARE(avg.at(1)-scan.positionsGlob.at(j).at(1));
			}
			std = sqrt(std/counter);


			//::std::cout << "Cluster " << c << ": [" << clusters.at(c).at(0) << ", " << clusters.at(c).at(1) << "]" << ::std::endl;
			std::vector<float> midpoint(2);
			unsigned int counter2 = 0;
			for(unsigned int j=clusters.at(c).at(0); j<=clusters.at(c).at(1); j++)
			{
				if(!isDynObstacle.at(j))
					continue;

				//::std::cout << "- " << scan.positionsGlob.at(j).at(0) << ", " << scan.positionsGlob.at(j).at(1) << ::std::endl;

				/*if(sqrt(dist.at(j-clusters.at(c).at(0))) > OUTLIER_STD_MULTIPLIER*std)
					continue;*/

				midpoint.at(0) += scan.positionsGlob.at(j).at(0);
				midpoint.at(1) += scan.positionsGlob.at(j).at(1);

				counter2++;
			}
			midpoint.at(0) /= counter2;
			midpoint.at(1) /= counter2;
			midpoint.at(0) *= 1000;
			midpoint.at(1) *= 1000;
			if(counter2 >= MIN_POINTS_IN_CLUSTER)
				clustersMid.push_back(midpoint);
		}


		// set latestObstacleBuffer
		{
			boost::mutex::scoped_lock l(m_mutex);
			this->latestObstacleBuffer.my_x = my_x;
			this->latestObstacleBuffer.my_y = my_y;
			this->latestObstacleBuffer.my_phi = my_phi;
			this->latestObstacleBuffer.obstacles = clustersMid;
		}
	}
}

bool LaserScanner::checkSignalStatus()
{
	bool wasPaused=false;
	boost::unique_lock<boost::mutex> lock(signal_mutex);
	while(signal == LaserScannerSignal::PAUSE) //wait if signal is PAUSE
	{
		wasPaused=true;
		//cout << "LaserScanner received signal PAUSED." << endl;
		FileLog::log_NOTICE("[LaserScanner] PAUSED");
		signal_cond.wait(lock); //waits for the notify, handles mutex locking/unlocking
	}

	if(wasPaused)
	{
		FileLog::log_NOTICE("[LaserScanner] RUN");
	}
	return true;
}


void LaserScanner::pause()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log_NOTICE("LaserScanner is signaled PAUSED.");
		signal=LaserScannerSignal::PAUSE;
	}
}


void LaserScanner::run()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log_NOTICE("LaserScanner is signaled RUN.");
		signal=LaserScannerSignal::RUN;
	}
	signal_cond.notify_all();
}

LaserScannerReadings LaserScanner::readings() const
{
	LaserScannerReadings readings;

	rec::robotino::api2::LaserRangeFinderReadings api_readings = scannerDriver->readings();
	readings.api_readings = api_readings;

	// compute angles, ranges and positions in local robotino frame
	unsigned int rangesSize = 0;
	const float* ranges;
	api_readings.ranges(&ranges, &rangesSize);

	readings.angles.resize(rangesSize);
	readings.ranges.resize(rangesSize);
	readings.inRange.resize(rangesSize);
	readings.positions.resize(rangesSize);
	readings.positionsGlob.resize(rangesSize);

	float cur_angle = api_readings.angle_min - api_readings.angle_increment;
	for(unsigned int i=0; i<rangesSize; ++i)
	{
		cur_angle += api_readings.angle_increment;

		// convert range and angle into point in laser coordinate system
		float x_laser, y_laser;
		range2vec2d(ranges[i], cur_angle, x_laser, y_laser);

		// transform to base coordinate system using calibration matrix
		float x_base, y_base;
		x_base = T_laser2base[0] * x_laser + T_laser2base[1] * y_laser + T_laser2base[2];
		y_base = T_laser2base[3] * x_laser + T_laser2base[4] * y_laser + T_laser2base[5];

		// convert point back to
		float range_base, angle_base;
		vec2d2range(x_base, y_base, range_base, angle_base);

		readings.angles[i] = angle_base;
		readings.ranges[i] = range_base;
		readings.inRange[i] = (ranges[i] > api_readings.range_min) && (ranges[i] < api_readings.range_max);
		readings.positions[i].resize(2);
		readings.positions[i][0] = x_base;
		readings.positions[i][1] = y_base;
		readings.positionsGlob[i].resize(2);
		readings.positionsGlob[i][0] = 0;
		readings.positionsGlob[i][1] = 0;
	}

	return readings;
}

void LaserScanner::rangeInBaseFrame(const float range, const float angle, float& base_range, float& base_angle) const
{
	// convert range and angle into point in laser coordinate system
	float x_laser, y_laser;
	range2vec2d(range, angle, x_laser, y_laser);

	// transform to base coordinate system using calibration matrix
	float x_base, y_base;
	x_base = T_laser2base[0] * x_laser + T_laser2base[1] * y_laser + T_laser2base[2];
	y_base = T_laser2base[3] * x_laser + T_laser2base[4] * y_laser + T_laser2base[5];

	// convert point back to
	vec2d2range(x_base, y_base, base_range, base_angle);
}

ObstacleBuffer LaserScanner::getLatestObstacleBuffer()
{
	boost::mutex::scoped_lock l(m_mutex);
	return this->latestObstacleBuffer;
}
