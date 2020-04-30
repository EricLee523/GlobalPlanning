#pragma once
#include <fstream>
#include <iomanip>
#include <opencv2/core.hpp>
#include <vector>

#define  EARTH_RADIUS   6378.137
#define  PI  3.1415926535897932

using namespace std;

struct node{
    double lat;
    double lng;
};

struct LEAD
{
    int id;
    double lng;
    double lat;
    int spd_limit;//限速
    int road_type;//道路类型	0,高速;1,路上;2,预路口;3,路口;4,非正常路,如越野;5,停车位;
    int cl_type;//换道类型	1,不可向左换道;2,不可向右换道;3,左右均不可换道;
    int lane_num;//车道数量
    int cur_lane;//当前车道
    int road_seg_id;//路段序号
    double heading;//航向
};

class CGetGPSData
{

public:
	CGetGPSData(void);
	~CGetGPSData(void);

    //int out_count;

public:

    /**
     * @brief 计算两个Gps点之间的距离
     * @param lat1 第一个点纬度
     * @param lng1 第一个经度
     * @param lat2 第二个点纬度
     * @param lng2  第二个点经度
     * @return 两点之间距离,单位:m
     */
	double GetDistance(double lat1, double lng1, double lat2, double lng2);


	CvPoint2D64f APiontConver(CvPoint2D64f v,CvPoint2D64f a,double direction);
	CvPoint2D64f APiontConverD(CvPoint2D64f v,CvPoint2D64f a,double direction);
    /**
     * @brief 计算到轨迹中点的横向距离
     * @param ori_gps 取轨迹中距离当前位置最近的一个点
     * @param ori_dir 该角度由origin_gps的前一个点和origin_gps计算得到
     * @param cur_gps 当前位置的gps值
     * @return 横向距离,单位:m
     */
	double LevelDist(CvPoint2D64f ori_gps, double ori_dir, CvPoint2D64f cur_gps);
    /**
     * @brief 计算到轨迹中点的纵向垂直距离
     * @param ori_gps 取轨迹中距离当前位置最近的一个点
     * @param ori_dir 该角度由origin_gps的前一个点和origin_gps计算得到
     * @param cur_gps 当前位置的gps值
     * @return 纵向垂直距离,单位:m
     */
	double VertDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps);
	CvPoint2D64f APiontConverG(CvPoint2D64f v,CvPoint2D64f a,double direction);
	CvPoint2D64f APiontConverP(CvPoint2D64f v,CvPoint2D64f a,double direction,CvPoint2D64f center,double rate);
	double GetAngle(double lat1, double lng1, double lat2, double lng2);
	double GetAngle(CvPoint2D64f a, CvPoint2D64f b);
	/**
	 * @brief 将角度转换为弧度
	 * @param d 角度
	 * @return 弧度,单位:rad
	 */
    double rad(double d) { return d * PI / 180.0; };
	CvPoint2D64f MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a);
  	void gps2plane(CvPoint2D64f origin,  vector<CvPoint2D64f> GPSPoints, double azimuth, vector<CvPoint2D64f> &planePoints);
  	void single2plane(CvPoint2D64f origin, CvPoint2D64f GPSPoints, double azimuth, CvPoint2D64f &planePoints);
  	void Bezier(CvPoint2D64f p[], int n, vector<CvPoint2D64f> &MPoint);
    /**
    * @brief 将世界坐标系转化成gps坐标值
    * @param origin_gps 世界坐标系为原点的gps值
    * @param world_point 世界坐标系下需要转换的点
    * @return 转换成gps后的点
    */
    CvPoint2D64f WorldPoint2Gps(CvPoint2D64f& origin_gps, const CvPoint2D64f& world_point);
    /**
     * @brief 将gps坐标值转化成世界坐标系
     * @param origin_gps 世界坐标系为原点的gps值
     * @param GPSPoint 需要转换的gps点
     * @return 转换成世界坐标系后的点
     */
    CvPoint2D64f Gps2WorldPoint(const CvPoint2D64f& origin_gps, CvPoint2D64f& GPSPoints);
    /**
     * @brief 轨迹平移的函数,默认向左平移
     * @param Origin_Path 原始路径
     * @param Move_Path 平移之后的路径
     * @param Move_Len 平移宽度
     */
    void TrajectoryMove(vector<node> Origin_Path, vector<node> &Move_Path, double Move_Len);
    /**
    * @brief 将一个点在坐标系下按照一个固定的角度旋转
    * @param origin_point 需要旋转的点
    * @param center_point 以该点为中心进行旋转
    * @param thetaz 旋转角度
    * @param out_point 旋转后得到的点
    * @param origin_gps 世界坐标系下原点
    */
    void RotatePoint(node origin_point, node center_point, double thetaz, node& out_point,  CvPoint2D64f &origin_gps);

    /**
     * @brief 以车道线的起点位为中心,按照一定角度旋转车道线
     * @param Origin_LaneLine 要旋转的车道线
     * @param Move_LaneLine 旋转后的车道线
     * @param off_angle 旋转角度
     * @param origin_gps 世界坐标系下原点
     */
    void RotateLaneLine(vector<node> Origin_LaneLine, vector<node> &Move_LaneLine, double off_angle,  CvPoint2D64f &origin_gps);

    void Bezier(CvPoint2D64f p[],int n,CvPoint2D64f *MPoint,int m);



};
