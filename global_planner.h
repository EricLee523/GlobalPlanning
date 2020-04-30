/*******************************************************************************
功    能：解析xml格式的openDrive高精度地图文件
创建时间：2019-09-02
作    者：lzy
当前版本: 1.0
********************************************************************************/

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "get_gps_data.h"


using namespace cv;
using namespace std;




struct Lane{
    int ID;
    int level = 1;
    bool is_uturn;
    float width; //车道宽度
    float offset;//相对上一车道偏移
    float angle;//相对于上一车道的角度,单位：rad
    int type;//线型   0:solid  1:broken

    vector<node> lane_node;//车道中间点:Gps
    vector<node> laneline_node;//车道线点:Gps

//    vector<CvPoint2D64f> lane_node_w;//车道中间点:world坐标系
//    vector<CvPoint2D64f> laneline_node_w;//车道线点:world坐标系

//    int successorID;
//    int presuccessorID;


};

struct ReferenceLine
{
    int type; //0:line 1:arc
    float curvature;
    CvPoint2D64f startPoint;
    CvPoint2D64f endPoint;
    float length;
    double  hdg;  //角度 单位:rad 正东方向为零
    vector<CvPoint2D64f> referLine_node_w;//参考车道线点:world坐标系

};

struct Connection_id
{
    int id;
    int id_num;
    vector<int> connect_road_id;
};

struct Road
{
    ReferenceLine referenceLine;
    vector<Lane> lanes;
    int id;
    int left_lane_num;//左侧车道数目
    int right_lane_num;//右侧车道数目
    int cur_left_lane_id;//当前路上左侧用于轨迹拼接的id
    int cur_right_lane_id;//当前路上右侧用于轨迹拼接的id
    bool is_oneway;//是否是单行道
    bool is_has_uturn;//是否含有uturn车道
    int road_type;//1:路上 2:预路口 3:路口
    float length; //道路长度
    int junction;
    int successorID;
    int successorType; //0:road 1:junction
    int presuccessorID;
    int presuccessorType; //0:road 1:junction
    //vector<int> connection_road_id;//用来存放路口关联的道路id
};


struct Way
{
    int id;
    int road_seg;//路段号
    int lane_num;
    float length;
    int road_type;
    int cur_lane_id;//当前路上用于轨迹拼接的id
    bool is_has_uturn;
    node start_point;//道路起点
    node end_point;//道路终点
    vector<Lane> lanes;
    vector<node> referLine_offset;//偏移出来的referLine,供全局规划用
    vector<int> connect_way_id;//可通向道路id
    vector<pair<int,int> > connect_way_id_index;//可通向道路id以及在my_way_vec_中的索引
};

struct Junction
{
    int id;
    int id_num;
    vector<int> incomingRoad;
    vector<int> connectingRoad;

};

class AstarNode
{
public:

    double way_id;//该节点所在道路的id
    int id;//使用vector来存储路点，id为vector中的节点号
    int parent_id;//方便提取最终的路点
    int road_type;//道路类型
    int road_seg;//路段号
    //int road_point_id;//从0开始，路上，预路口，路口路段内节点的开始路点号
    //int road_id_end;//一段路段的最后一个点
    //方便提取最终的路点_end
    double lat;//纬度
    double lng;//经度
    int childNum;//留有余量，目前孩子节点最多也就3个
    int child[6];
    double disToChild[6];//由父节点到孩子节点的距离
    double  g;
    double h;
    double f;
    vector<pair<int,int> > connect_way_id_index;//可通向道路id以及在map_data_中的索引
//    //判断是否为带有预路口道路上的节点
//    bool isbfCrossing ;
    AstarNode()
    {
        parent_id = -2;//初始化parID，赋值为-2；
        childNum =0;
        way_id = -1;

    }

};



class GlobalPlanner {

public:
    GlobalPlanner();
    ~GlobalPlanner();
    /**
     * @brief 初始化
     */
    void Init();
    /**
     * @brief 全局路径规划主接口
     * @return true:成功规划全局路径
     */
    bool searchPath();
    /**
     * @brief 输出规划出来的最优路径在map_data_中的依次索引
     */
    void GetMdf();
    /**
     * @brief 全局轨迹拼接
     */
    void GetGlobalPath();
    /**
     * @brief 重置一些变量
     */
    void Reset();

private:
    /**
     * @brief 读取xml格式的openDrive高精度地图文件,将其解析到数据结构中
     * @param m_xml 文件的路径
     */
    void ReadFromXml(const char* m_xml);
    /**
     * @brief 将解析好的高精度地图文件中的世界坐标点,转化成gps的形式,并制作车道线和车道中心线
     */
    void Convert2Gps();
    /**
     * @brief 添加正、反向道路,
     */
    void ConstructRoadMap();

    /**
     * @brief 创建预路口后的关联路段
     */
    void ConstructConnectRoad();

    /**
     * @brief 创建道路之间的拓扑关系，将其存储到map_data中
     * @return true:创建成功 false:创建失败
     */
    bool ConstructGlobalMap();
    /**
     * @brief astar算法主函数
     * @param tmp_goal_node_id 正确的goal_node_id
     * @return true:成功找到最优路径
     */
    bool solve(int tmp_goal_node_id);
    /**
     * @brief 判断AstarNode的f值的大小
     * @param left
     * @param right
     * @return true:left的f值小于right的f值
     */
    static bool compare(const AstarNode& left,const AstarNode& right);
    /**
     * @brief 处理当前节点的所有孩子节点
     * @param current 当前节点
     * @param goal_node 目标节点
     */
    void NextStep(AstarNode & current, AstarNode &goal_node);
    /**
     * @brief 处理当前节点
     * @param current 大当前节点
     * @param childNode 孩子节点
     * @param g 节点的g值
     * @param goal_node 目标节点
     */
    void checkNode( AstarNode & current, AstarNode &childNode,double g, AstarNode &goal_node);
    /**
     * @brief 检查NodeList中有没有包含node
     * @param Nodelist
     * @param node
     * @return 找到该node返回该node在Nodelist中的索引值
     */
    int isContains(const vector<AstarNode> & Nodelist, AstarNode & node);
    /**
     * @brief 根据road的id获取在my_way_vec中的索引
     * @param id road的一个id
     * @return my_way_vec中的索引
     */
    int GetIdIndex(const int& id);

     /**
      * @brief 获取下一段路的方向
      * @param back_path 前一条路
      * @param forth_path 后一条路
      * @return 1：直行 2：左转 3：右转 4:uturn
      */
    int GetDirection(vector<node> &back_path, vector<node> &forth_path);
    /**
     * @brief 获取地图的边界
     */
    void GetMapBoundary();

    /**
     * @brief 换道曲线拟合
     * @param cur_way 当前道路
     * @param next_way 下一条路
     * @param insert_node 拟合后的点
     */
    void CurveFittingLaneChange(vector<node> & cur_way, vector<node> &next_way, vector<node> &insert_node);
    /**
     * @brief 左、右转曲线拟合
     * @param cur_way 当前道路
     * @param next_way 下一条路
     * @param insert_node 拟合后的点
     */
    void CurveFittingLRturn(vector<node> & cur_way, vector<node> &next_way, vector<node> &insert_node);
    /**
     * @brief uturn曲线拟合
     * @param cur_way 当前道路
     * @param next_way 下一条路
     * @param insert_node 拟合后的点
     */
    void CurveFittingUturn(vector<node> & cur_way, vector<node> &next_way, vector<node> &insert_node);
    /**
     * @brief 预路口前轨迹拟合
     * @param pre_node1 预路口前一段路的倒数第二个点
     * @param pre_node2 预路口前一段路的最后一个点
     * @param next_node3 预路口的第一个点
     * @param next_node4 预路口的第二个点
     * @param insert_node 拟合后的点
     */
    void CurveFittingPreJunction(node pre_node1, node pre_node2,node next_node3,node next_node4, vector<node> &insert_node);


public:
    vector<Way>my_way_vec_;//将my_roads_vec_处理后，用来做全局规划
    vector<AstarNode> map_data_;
    int start_node_id_ ;//起始节点在map_data_中的索引
    int goal_node_id_ ;//目标节点在map_data_中的索引
    vector<int> glo_path_index_;//全局规划路线在map_data_中的索引

    //图像边界
    double left_ = 360;
    double right_ = 0;
    double top_ = 360;
    double bottom_ = 0;

private:
    vector<Road> my_roads_vec_;//HDmap中解析后的内容
    CvPoint2D64f origin_gps_;
    vector<Junction> my_junction_vec_;
    vector<Connection_id> my_connection_id_vec_;
    CGetGPSData my_gps_data_;

    vector<AstarNode> open_list_;//开放列表
    vector<AstarNode> close_list_;//封闭列表
    AstarNode start_node_;//起始节点
    AstarNode goal_node_;//目标节点
    int right_goal_node_id_;//最后正确的目标节点在map_data_中的索引
    vector<int> mdf_path_;//全局规划路线在my_way_vec_中的索引
    vector<int> lead_turn_dir_vec_;//全局轨迹每条路的行驶方向 1：直行 2：左转 3：右转 4:uturn
    vector<LEAD> glo_path_vec_;//全局轨迹的数据

    ofstream *out_file_;//输出文件,用于调试



    char flie_name_[20] = "./file/HDmap.xml";//高精度地图文件
//    char flie_name_[20] = "./file/lzy_HDmap.xml";//高精度地图文件


};


#endif //GLOBAL_PLANNER_H
