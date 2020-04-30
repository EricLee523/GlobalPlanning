#include "get_gps_data.h"
using namespace std;

CGetGPSData::CGetGPSData(void)
{

}

CGetGPSData::~CGetGPSData(void)
{

}

/*********************************************************************************************
根据经纬度坐标计算实际两点距离
输入：
lat1,lng1:第一点纬度、经度
lat2,lng2:第二点纬度、经度
输出：
返回两点距离
********************************************************************************************/
double CGetGPSData::GetDistance(double lat1, double lng1, double lat2, double lng2)
{
	if(lat1 == lat2 && lng1 == lng2)
		return 0;
    double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = rad(lng1) - rad(lng2);
    double s = 2 * asin(sqrt(pow(sin(a/2),2) +
							 cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS*1000;
    //s = round(s * 10000) / 10000;
    return s;
}

//根据两点经纬度求方向，与北夹角
double CGetGPSData::GetAngle(CvPoint2D64f apoint, CvPoint2D64f bpoint)
{
	double lat1 = apoint.x;
	double lng1 = apoint.y;
	double lat2 = bpoint.x;
	double lng2 = bpoint.y;
	double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = (rad(lng1) - rad(lng2))*cos(radLat1);
	double t=atan(b/a);
	if (a<0)
		t=t+PI;
	//t=PI/2-t;

	t = t * 180 / PI;
	return t;
}

/*********************************************************************************************
根据经纬度坐标计算实际两点距离
输入：
  lat1,lng1:第一点纬度、经度
  lat2,lng2:第二点纬度、经度
输出：
  返回两点夹角，与正北方向夹角，顺时针正，逆时针负,单位度
********************************************************************************************/
double CGetGPSData::GetAngle(double lat1, double lng1, double lat2, double lng2)
{
	//与北夹角
	if(lat1 == lat2 && lng1 == lng2)
		return 0;
	double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = (rad(lng1) - rad(lng2))*cos(radLat1);
	double t=atan(b/a);
	if (a<0)
	{
		t=t+PI;

	}

	t = t * 180 / PI;
	return t;
}
/**
 * @brief 计算到轨迹中点的横向距离
 * @param ori_gps 取轨迹中距离当前位置最近的一个点
 * @param ori_dir 该角度由origin_gps的前一个点和origin_gps计算得到
 * @param cur_gps 当前位置的gps值
 * @return 横向距离,单位:m
 */
double CGetGPSData::LevelDist(CvPoint2D64f ori_gps, double ori_dir, CvPoint2D64f cur_gps)
{
	double north_dir = GetAngle(cur_gps,ori_gps);
	double head_path_angle = (north_dir - ori_dir)*PI/180;

	double dist = GetDistance(cur_gps.x, cur_gps.y, ori_gps.x, ori_gps.y);
	dist = abs(dist*sin( head_path_angle ));
	return dist;
}

/*
目标点的GPS坐标转换为地图坐标。
参数：v - 车的GPS坐标
     a - 目标点GPS坐标
   direction - 车的航向
*/
CvPoint2D64f CGetGPSData::APiontConver(CvPoint2D64f v,CvPoint2D64f a,double direction)
{
	double t = GetAngle(a.x,a.y,v.x,v.y);
	double m = direction - t +90;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	CvPoint2D64f p;
	p.x = 250 + (int)(s * cos(m) * 5);  //*5换成20cm单位
	p.y = 500 - (int)(s * sin(m) * 5);

	return p;

}

CvPoint2D64f CGetGPSData::APiontConverD(CvPoint2D64f v,CvPoint2D64f a,double direction)
{
	double t = GetAngle(a.x,a.y,v.x,v.y);
	double m = direction - t + 90;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	CvPoint2D64f p;
	p.x = 250 + (s * cos(m) * 5);  //*5换成20cm单位
	p.y = 500 - (s * sin(m) * 5);

	return p;
}
/**
 * @brief 计算到轨迹中点的纵向垂直距离
 * @param ori_gps 取轨迹中距离当前位置最近的一个点
 * @param ori_dir 该角度由origin_gps的前一个点和origin_gps计算得到
 * @param cur_gps 当前位置的gps值
 * @return 纵向垂直距离,单位:m
 */
double CGetGPSData::VertDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps)
{
	double north_dir = GetAngle(cur_gps,ori_gps);
	double head_path_angle = (north_dir - ori_dir)*PI/180;

	double dist = GetDistance(cur_gps.x,cur_gps.y,ori_gps.x, ori_gps.y);
	dist = dist*cos( head_path_angle );

	return dist;
}


CvPoint2D64f CGetGPSData::APiontConverG(CvPoint2D64f v,CvPoint2D64f a,double direction)
{
	double t = GetAngle(a.x,a.y,v.x,v.y);
	double m = direction - t + 90;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	CvPoint2D64f p;
	p.x = 250 + (s * cos(m)/2.75);  //除以8换成8.5m单位
	p.y = 250 - (s * sin(m)/2.75);

	return p;
}

CvPoint2D64f CGetGPSData::APiontConverP(CvPoint2D64f v,CvPoint2D64f a,double direction,CvPoint2D64f center,double rate)
{
	double t = GetAngle(a.x,a.y,v.x,v.y);
	double m = direction - t + 90;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	CvPoint2D64f p;
	p.x = center.x + (s * cos(m)/rate);  //除以8换成8.5m单位
	p.y = center.y - (s * sin(m)/rate);

	return p;
}

//地图坐标转GPS
CvPoint2D64f CGetGPSData::MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a)
{
	if (a.x == 250&&a.y == 500)
	{
		return v;
	}
	double xq1,yq1,x,y,sita,dire,xa,ya,x1,y1;
	double s1;
	xq1 = a.x-250;
	yq1 = 500-a.y;
	s1=sqrt(pow(xq1/5,2)+pow(yq1/5,2));
    sita=atan(xq1/yq1);
	if(yq1<0)
		sita += PI;
	dire=sita+rad(dir);
	ya=s1*sin(dire);
	xa=s1*cos(dire);
	x=v.x+(xa*180)/(6378137*3.1415926);
	y=v.y+(ya*180)/(6378137*3.1415926*cos(x*3.1415926/180));
	CvPoint2D64f c;
	c.x=x;
	c.y=y;
	return c;
}

void CGetGPSData::gps2plane(CvPoint2D64f origin,  vector<CvPoint2D64f> GPSPoints, double azimuth, vector<CvPoint2D64f> &planePoints)
{
	double lat0 = origin.y;
	double lng0 = origin.x;
	planePoints.clear();
	for (int i = 0; i < GPSPoints.size(); i++)
	{
		double lat = GPSPoints[i].y;
		double lng = GPSPoints[i].x;
		double X = 6371004.0 * cos(lat * PI / 180.0) * 2.0 * PI / 360.0 * (lng - lng0);
		double Y = 6371004.0 * (lat - lat0) * PI / 180.0;

//            double b=Y * sin(azimuth * PI / 180.0);
//            double c=X * cos(azimuth * PI / 180.0);
		CvPoint2D64f pt;
		pt.x = Y * cos(azimuth * PI / 180.0) + X * sin(azimuth * PI / 180.0);
		pt.y = Y * sin(azimuth * PI / 180.0) - X * cos(azimuth * PI / 180.0);
		planePoints.push_back(pt);
	}
}

void CGetGPSData::single2plane(CvPoint2D64f origin, CvPoint2D64f GPSPoints, double azimuth, CvPoint2D64f &planePoints)
{
    double lat0 = origin.y;
    double lng0 = origin.x;

    double lat = GPSPoints.y;
    double lng = GPSPoints.x;
    double X = 6371004.0 * cos(lat * PI / 180.0) * 2.0 * PI / 360.0 * (lng - lng0);
    double Y = 6371004.0 * (lat - lat0) * PI / 180.0;

    planePoints.x = Y * cos(azimuth * PI / 180.0) + X * sin(azimuth * PI / 180.0);
    planePoints.y = Y * sin(azimuth * PI / 180.0) - X * cos(azimuth * PI / 180.0);

}

void CGetGPSData::Bezier(CvPoint2D64f p[], int n, vector<CvPoint2D64f> &MPoint)
{

    MPoint.clear();
    CvPoint2D64f *pc=new CvPoint2D64f[n+1];
    int i,r;
    double u;
    int count = 0;
    //u的步长决定了曲线点的精度
    for(int k = 0;k<200;k++)
    {
        u=(double)k/199.0;
        for(i=0;i<=n;i++)pc[i]=p[i];

        for(r=1;r<=n;r++)
        {
            for(i=0;i<=n-r;i++)
            {
                pc[i].x=(1-u)*pc[i].x+u*pc[i+1].x;
                pc[i].y=(1-u)*pc[i].y+u*pc[i+1].y;
            }
        }
        MPoint.push_back(pc[0]);
    }

    delete [] pc;
}



/**
 * @brief 将世界坐标系转化成gps坐标值
 * @param origin_gps 世界坐标系为原点的gps值
 * @param world_point 世界坐标系下需要转换的点
 * @return 转换成gps后的点
 */
CvPoint2D64f CGetGPSData::WorldPoint2Gps(CvPoint2D64f& origin_gps, const CvPoint2D64f& world_point)
{
//    double dx = world_point.x;
//    double dy = world_point.y;
//    double lat0 = origin_gps.x;
//    double lng0 = origin_gps.y;
//    double lat = dy * 180.0 / (PI * EARTH_RADIUS * 1000 ) + lat0;
//    double lng = dx * 180.0 /(EARTH_RADIUS *1000 * cos(lat * PI / 180.0) * PI) + lng0;
//
//    CvPoint2D64f tmp_gps;
//    tmp_gps.x = lat;
//    tmp_gps.y = lng;
//    return tmp_gps;

    double x = world_point.x;
    double y = world_point.y;
    double lat0 = origin_gps.x;;
    double lng0 = origin_gps.y;
    double lat = y*180/6371004/3.1415926 + lat0;
    double lng = x*180/6371004/3.1415926/cos(lat*3.1415926/180) + lng0;

    CvPoint2D64f tmp_gps;
    tmp_gps.x = lat;
    tmp_gps.y = lng;
    return tmp_gps;


}


/**
 * @brief 将gps坐标值转化成世界坐标系
 * @param origin_gps 世界坐标系为原点的gps值
 * @param GPSPoint 需要转换的gps点
 * @return 转换成世界坐标系后的点
 */
CvPoint2D64f CGetGPSData::Gps2WorldPoint(const CvPoint2D64f& origin_gps, CvPoint2D64f& GPSPoint)
{
//	double lat0 = origin_gps.x;
//	double lng0 = origin_gps.y;
//
//	double lat = GPSPoint.x;
//	double lng = GPSPoint.y;
//	double X = EARTH_RADIUS * 1000 * cos(lat * PI / 180.0) * PI  * (lng - lng0) / 180.0;
//	double Y = EARTH_RADIUS * 1000 * (lat - lat0) * PI / 180.0;
//
//	CvPoint2D64f tmp_world_pt;
//	tmp_world_pt.x = X;
//	tmp_world_pt.y = Y;
//	return tmp_world_pt;


    double lat = GPSPoint.x;
    double lng = GPSPoint.y;
    double lat0 = origin_gps.x;;
    double lng0 = origin_gps.y;
    double x = 6371004*cos(lat*3.1415926/180)*2*3.1415926/360*(lng - lng0);

    double y = 6371004*(lat - lat0)*3.1415926/180;

    CvPoint2D64f tmp_world_pt;
	tmp_world_pt.x = x;
	tmp_world_pt.y = y;
	return tmp_world_pt;

}

/**
 * @brief 轨迹平移的函数,默认向左平移
 * @param Origin_Path 原始路径
 * @param Move_Path 平移之后的路径
 * @param Move_Len 平移宽度
 */
void CGetGPSData::TrajectoryMove(vector<node> Origin_Path, vector<node> &Move_Path, double Move_Len)
{
	Move_Path.clear();
	for(int i = 0;i<Origin_Path.size()-1;i++)
	{
		double dir1 = GetAngle(Origin_Path[i+1].lat, Origin_Path[i+1].lng, Origin_Path[i].lat, Origin_Path[i].lng);
		double dir2 = dir1 - 90;
		double rad_dir2 = dir2 * PI / 180.0;

		double x = Move_Len * cos(rad_dir2);
		double y = Move_Len * sin(rad_dir2);

		double origin_lat = Origin_Path[i].lat;

		double dx = (x*180.0)/(6378137 * PI);
		double dy = (y*180.0)/(6378137 * PI * cos(origin_lat*PI/180.0));

		node pt;
		pt.lat = Origin_Path[i].lat + dx;
		pt.lng = Origin_Path[i].lng + dy;

		Move_Path.push_back(pt);

		if(i == Origin_Path.size()-2)
		{
			pt.lat = Origin_Path[i+1].lat + dx;
			pt.lng = Origin_Path[i+1].lng + dy;

			Move_Path.push_back(pt);
		}
	}
}

/**
 * @brief 将一个点在坐标系下按照一个固定的角度旋转
 * @param origin_point 需要旋转的点
 * @param center_point 以该点为中心进行旋转
 * @param thetaz 旋转角度
 * @param out_point 旋转后得到的点
 * @param origin_gps 世界坐标系下原点
*/
void CGetGPSData::RotatePoint(node origin_point, node center_point, double thetaz, node& out_point,  CvPoint2D64f &origin_gps) {
    CvPoint2D64f tmp_gpspoint, origin_world_pt, center_world_point, out_world_pt;
    tmp_gpspoint.x = origin_point.lat;
    tmp_gpspoint.y = origin_point.lng;
    origin_world_pt = Gps2WorldPoint(origin_gps, tmp_gpspoint);
    tmp_gpspoint.x = center_point.lat;
    tmp_gpspoint.y = center_point.lng;
    center_world_point = Gps2WorldPoint(origin_gps, tmp_gpspoint);

    double x1 = origin_world_pt.x - center_world_point.x;
    double y1 = origin_world_pt.y - center_world_point.y;
    double rz = thetaz * CV_PI / 180;
    out_world_pt.x = cos(rz) * x1 - sin(rz) * y1 + center_world_point.x;
    out_world_pt.y = sin(rz) * x1 + cos(rz) * y1 + center_world_point.y;

    tmp_gpspoint = WorldPoint2Gps(origin_gps, out_world_pt);

    out_point.lat = tmp_gpspoint.x;
    out_point.lng = tmp_gpspoint.y;

}



/**
 * @brief 简化版的旋转坐标点的函数,不是很准,但是够用
 */
//void CGetGPSData::RotatePoint(node origin_point, node center_point, double thetaz, node& out_point,  CvPoint2D64f &origin_gps)
//{
//    double x1 =  origin_point.lat - center_point.lat;
//    double y1 =  origin_point.lng - center_point.lng;
//    double rz = -thetaz * CV_PI / 180;
//    out_point.lat = cos(rz) * x1 - sin(rz) * y1 + center_point.lat;
//    out_point.lng = sin(rz) * x1 + cos(rz) * y1 + center_point.lng;
//
//}


/**
 * @brief 以车道线的起点位为中心,按照一定角度旋转车道线
 * @param Origin_LaneLine 要旋转的车道线
 * @param Move_LaneLine 旋转后的车道线
 * @param off_angle 旋转角度
 * @param origin_gps 世界坐标系下原点
 */
void CGetGPSData:: RotateLaneLine(vector<node> Origin_LaneLine,  vector<node> &Move_LaneLine, double off_angle,  CvPoint2D64f &origin_gps)
{
	Move_LaneLine.clear();
	node tmp_node_in, tmp_node_out,center_point;
	tmp_node_out.lat = Origin_LaneLine[0].lat;
	tmp_node_out.lng = Origin_LaneLine[0].lng;
	center_point.lat = Origin_LaneLine[0].lat;
	center_point.lng = Origin_LaneLine[0].lng;
	Move_LaneLine.push_back(tmp_node_out);

	for(int i = 1;i<Origin_LaneLine.size();i++)
	{
		tmp_node_in.lat = Origin_LaneLine[i].lat;
		tmp_node_in.lng = Origin_LaneLine[i].lng;

        RotatePoint(tmp_node_in,center_point, off_angle, tmp_node_out,origin_gps);
		Move_LaneLine.push_back(tmp_node_out);
	}

}

void CGetGPSData::Bezier(CvPoint2D64f p[],int n,CvPoint2D64f *MPoint,int m)
{

	CvPoint2D64f *pc=new CvPoint2D64f[n+1];
	int i,r;
	double u;
	int count = 0;
	//u的步长决定了曲线点的精度
	for(int k = 0;k<m;k++)
	{
		u=k/double(m-1);
		for(i=0;i<=n;i++)pc[i]=p[i];

		for(r=1;r<=n;r++)
		{
			for(i=0;i<=n-r;i++)
			{
				pc[i].x=(1-u)*pc[i].x+u*pc[i+1].x;
				pc[i].y=(1-u)*pc[i].y+u*pc[i+1].y;
			}
		}
		MPoint[k] = pc[0];

	}
	delete [] pc;

}