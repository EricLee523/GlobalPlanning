/*******************************************************************************
功    能：解析xml格式的openDrive高精度地图文件
创建时间：2019-09-02
作    者：lzy
当前版本: 1.0
********************************************************************************/

#include "global_planner.h"
#include<iomanip>


template <class Type>
Type charToNum(char* char_value)
{
    string str;
    str = char_value;
    istringstream iss(str);
    Type num;
    iss >> num;
    return  num;
}

GlobalPlanner::GlobalPlanner() {
    
    out_file_ = new ofstream("/home/lzy/file2/road_gps_file/outfile.txt");

}
GlobalPlanner::~GlobalPlanner()
{
    delete out_file_;
}
/**
 * @brief 初始化
 */
void GlobalPlanner::Init()
{
    ReadFromXml(flie_name_);
    Convert2Gps();
    ConstructRoadMap();
    ConstructConnectRoad();
    GetMapBoundary();
    ConstructGlobalMap();
//    searchPath();
//    GetMdf();
//    GetGlobalPath();
}

/**
     * @brief 重置一些变量
     */
void GlobalPlanner::Reset()
{
    open_list_.clear();
    close_list_.clear();
    glo_path_index_.clear();
    mdf_path_.clear();
    lead_turn_dir_vec_.clear();
    glo_path_vec_.clear();
}

/**
 * @brief 读取xml格式的openDrive高精度地图文件,将其解析到数据结构中
 * @param m_xml 文件的路径
 */
void GlobalPlanner::ReadFromXml(const char* m_xml){
    cout<<"readXml"<<endl;

    xmlDocPtr pdoc = NULL;
    xmlNodePtr proot = NULL, pcur = NULL;
/*****************打开xml文档********************/
    xmlKeepBlanksDefault(0);//必须加上，防止程序把元素前后的空白文本符号当作一个node
    pdoc = xmlReadFile (m_xml, "UTF-8", XML_PARSE_RECOVER);//libxml只能解析UTF-8格式数据

    if (pdoc == NULL)
    {
        cout<<"error:can't open file!"<<endl;
        exit (1);
    }

/*****************获取xml文档对象的根节对象********************/
    proot = xmlDocGetRootElement (pdoc); //根节点<OpenDRIVE>

    if (proot == NULL)
    {
        cout<<"error: file is empty!"<<endl;
        exit (1);
    }

/*****************查找根节点<OpenDRIVE>下所有节点********************/
    pcur = proot->xmlChildrenNode;   //header节点

/*****************解析header节点中的内容********************/
    if (pcur != NULL)
    {
        _xmlAttr *ptr = pcur->properties;
        while(ptr != NULL)
        {
            char* tempname = (char *)ptr->name;
            string s = tempname;
            if(s == "north")
            {
                char* lat_char = (char *)ptr->children->content;
                double lat = charToNum<double>(lat_char);
                origin_gps_.x = lat;//坐标原点纬度
                //cout<<setprecision(9) <<lat<<endl;
            }
            if(s == "east")
            {
                char* lon_char = (char *)ptr->children->content;
                double lon = charToNum<double>(lon_char);
                origin_gps_.y = lon;//坐标原点经度
                //cout<<setprecision(9) <<lon<<endl;
            }
            ptr = ptr->next;
        }

    }
/*****************解析road节点中的内容********************/
    pcur= pcur->next;    //road节点
    char* m_value_char;
    while (pcur != NULL && !xmlStrcmp(pcur->name, BAD_CAST("road")))
    {
        node m_node;
        Road m_road;
        Lane m_lane;
        /*****************road标签中的信息********************/
        _xmlAttr *ptr_road = pcur->properties;
        ptr_road = ptr_road->next;//length
        m_value_char = (char*)ptr_road->children->content;
        m_road.length = charToNum<float>(m_value_char);

        ptr_road = ptr_road->next;//id
        m_value_char = (char*)ptr_road->children->content;
        m_road.id = charToNum<int>(m_value_char);

        ptr_road = ptr_road->next;//junction
        m_value_char = (char*)ptr_road->children->content;
        m_road.junction = charToNum<int>(m_value_char);

        ptr_road = ptr_road->next;//roadtype
        m_value_char = (char*)ptr_road->children->content;
        m_road.road_type = charToNum<int>(m_value_char);

        ptr_road = ptr_road->next;//rightId
        m_value_char = (char*)ptr_road->children->content;
        m_road.cur_right_lane_id = charToNum<int>(m_value_char);

        ptr_road = ptr_road->next;//leftId
        m_value_char = (char*)ptr_road->children->content;
        m_road.cur_left_lane_id = charToNum<int>(m_value_char);


        /*****************link标签中的信息********************/
        xmlNodePtr ptr_link=pcur->xmlChildrenNode;//link节点
        if (ptr_link != NULL && !xmlStrcmp(ptr_link->name, BAD_CAST("link")))
        {
            xmlNodePtr ptr = ptr_link->xmlChildrenNode;//successor节点
            while(ptr != NULL)
            {
                m_value_char = (char *)ptr->name;
                string s = m_value_char;
                if(s == "successor")
                {
                    _xmlAttr *ptr_success_tag = ptr->properties;//指向elementType
                    char* success_type_char = (char*)ptr_success_tag->children->content;
                    string success_type_str = success_type_char;
                    if(success_type_str == "road")
                    {
                        m_road.successorType = 0;//road
                    }else if(success_type_str == "junction")
                    {
                        m_road.successorType = 1;//junction
                    }
                    ptr_success_tag = ptr_success_tag->next;//指向elementId
                    char* success_id_char = (char *)ptr_success_tag->children->content;
                    int success_id = charToNum<int>(success_id_char);
                    m_road.successorID = success_id;
                }
                if(s == "predecessor")
                {
                    _xmlAttr *ptr_presuccess_tag = ptr->properties;
                    char* presuccess_type_char = (char*)ptr_presuccess_tag->children->content;
                    string presuccess_type_str = presuccess_type_char;
                    if(presuccess_type_str == "road")
                    {
                        m_road.presuccessorType = 0;//road
                    }else if(presuccess_type_str == "junction")
                    {
                        m_road.presuccessorType = 1;//junction
                    }
                    ptr_presuccess_tag = ptr_presuccess_tag->next;//指向elementId
                    char* presuccess_id_char = (char *)ptr_presuccess_tag->children->content;
                    int presuccess_id = charToNum<int>(presuccess_id_char);
                    m_road.presuccessorID = presuccess_id;
                }
                ptr = ptr->next;
            }

        }
        /*****************planView标签中的信息********************/
        xmlNodePtr ptr_planView = ptr_link->next; //planView节点
        if (ptr_planView != NULL && !xmlStrcmp(ptr_planView->name, BAD_CAST("planView")))
        {

            xmlNodePtr ptr = ptr_planView->xmlChildrenNode;//geometry节点
            if(ptr != NULL && !xmlStrcmp(ptr->name, BAD_CAST("geometry")))
            {

                _xmlAttr *ptr_geometry_tag = ptr->properties;
                ptr_geometry_tag = ptr_geometry_tag->next;//指向x
                m_value_char = (char*)ptr_geometry_tag->children->content;
                m_road.referenceLine.startPoint.x = charToNum<double >(m_value_char);

                ptr_geometry_tag = ptr_geometry_tag->next;//指向y
                m_value_char = (char*)ptr_geometry_tag->children->content;
                m_road.referenceLine.startPoint.y = charToNum<double>(m_value_char);

                ptr_geometry_tag = ptr_geometry_tag->next;//指向hdg
                m_value_char = (char*)ptr_geometry_tag->children->content;
                m_road.referenceLine.hdg = charToNum<double>(m_value_char);

                ptr_geometry_tag = ptr_geometry_tag->next;//指向length
                m_value_char = (char*)ptr_geometry_tag->children->content;
                m_road.referenceLine.length = charToNum<float>(m_value_char);



                xmlNodePtr ptr_type = ptr->xmlChildrenNode;//指向线型:line or arc
                if(!xmlStrcmp(ptr_type->name, BAD_CAST("line")) )
                {
                    m_road.referenceLine.type = 0;
                    m_road.referenceLine.curvature = 0;
                } else{
                    ptr_geometry_tag = ptr_type->properties;
                    m_value_char = (char*)ptr_geometry_tag->children->content;
                    m_road.referenceLine.curvature = charToNum<float>(m_value_char);
                    m_road.referenceLine.type = 1;
                }
            }

        }
        /*****************lanes标签中的信息********************/
        xmlNodePtr ptr_lanes = ptr_planView->next; //lanes节点
        if (ptr_lanes != NULL && !xmlStrcmp(ptr_lanes->name, BAD_CAST("lanes")))
        {
            ptr_lanes = ptr_lanes->xmlChildrenNode;//laneSection标签
            ptr_lanes = ptr_lanes->xmlChildrenNode;//center标签
            while(ptr_lanes != NULL)
            {
                if(!xmlStrcmp(ptr_lanes->name, BAD_CAST("center")))
                {
                    xmlNodePtr ptr_center = ptr_lanes->xmlChildrenNode;//lane标签
                    _xmlAttr *ptr_center_lane_tag = ptr_center->properties;
                    m_value_char = (char*)ptr_center_lane_tag->children->content;
                    m_lane.ID = charToNum<int >(m_value_char);

                    ptr_center_lane_tag = ptr_center_lane_tag->next->next;//level标签
                    m_value_char = (char*)ptr_center_lane_tag->children->content;
                    m_lane.level = charToNum<int >(m_value_char);

                    ptr_center_lane_tag = ptr_center_lane_tag->next;//uturn标签
                    m_value_char = (char*)ptr_center_lane_tag->children->content;
                    int m_uturn = charToNum<int >(m_value_char);
                    if(m_uturn == 1)
                    {
                        m_lane.is_uturn = true;
                    } else{
                        m_lane.is_uturn = false;
                    }


                    ptr_center = ptr_center->xmlChildrenNode;//link标签
                    ptr_center = ptr_center->next;//roadMark标签
                    ptr_center_lane_tag = ptr_center->properties;//指向sOffset
                    ptr_center_lane_tag = ptr_center_lane_tag->next;//指向type

                    if(!xmlStrcmp(ptr_center_lane_tag->children->content, BAD_CAST("solid")))
                    {
                        m_lane.type = 0;
                    }else if (!xmlStrcmp(ptr_center_lane_tag->children->content, BAD_CAST("broken")))
                    {
                        m_lane.type = 1;
                    }
                    m_lane.offset = 0;
                    m_lane.width = 0;
                    m_lane.angle = 0;
                    m_road.left_lane_num = 0;
                    m_road.right_lane_num = 0;
                    m_road.is_has_uturn = false;
                    m_road.lanes.push_back(m_lane);
                }else if(!xmlStrcmp(ptr_lanes->name, BAD_CAST("right")))
                {
                    xmlNodePtr ptr_right = ptr_lanes->xmlChildrenNode;//lane标签
                    xmlNodePtr ptr_right_lane;
                    while(ptr_right != NULL )
                    {
                        _xmlAttr *ptr_right_lane_tag = ptr_right->properties;
                        m_value_char = (char*)ptr_right_lane_tag->children->content;//id标签
                        m_lane.ID = charToNum<int >(m_value_char);
                        m_road.right_lane_num = -1 * m_lane.ID;//计算右侧道路条数

                        ptr_right_lane_tag = ptr_right_lane_tag->next->next;//level标签
                        m_value_char = (char*)ptr_right_lane_tag->children->content;
                        m_lane.level = charToNum<int>(m_value_char);

                        ptr_right_lane_tag = ptr_right_lane_tag->next;//uturn标签
                        m_value_char = (char*)ptr_right_lane_tag->children->content;
                        int m_uturn = charToNum<int >(m_value_char);
                        if(m_uturn == 1)
                        {
                            m_lane.is_uturn = true;
                            m_road.is_has_uturn = true;
                        } else{
                            m_lane.is_uturn = false;
                        }

                        ptr_right_lane = ptr_right->xmlChildrenNode;//link标签
                        ptr_right_lane = ptr_right_lane->next;//width标签
                        ptr_right_lane_tag = ptr_right_lane->properties;//sOffset标签

                        ptr_right_lane_tag = ptr_right_lane_tag->next;//a标签,偏移距离
                        m_value_char = (char*)ptr_right_lane_tag->children->content;
                        m_lane.offset = charToNum<float>(m_value_char);
                        m_lane.width = m_lane.offset;//偏移距离和道路宽度一致

                        ptr_right_lane_tag = ptr_right_lane_tag->next;//b标签,偏移角度
                        m_value_char = (char*)ptr_right_lane_tag->children->content;
                        m_lane.angle = charToNum<float>(m_value_char);

                        ptr_right_lane = ptr_right_lane->next;//roadMark标签
                        ptr_right_lane_tag = ptr_right_lane->properties;//sOffset标签
                        ptr_right_lane_tag = ptr_right_lane_tag->next;//指向type

                        if(!xmlStrcmp(ptr_right_lane_tag->children->content, BAD_CAST("solid")))
                        {
                            m_lane.type = 0;
                        }else if (!xmlStrcmp(ptr_right_lane_tag->children->content, BAD_CAST("broken")))
                        {
                            m_lane.type = 1;
                        }
                        m_road.lanes.push_back(m_lane);

                        ptr_right = ptr_right->next;
                    }
                }else if(!xmlStrcmp(ptr_lanes->name, BAD_CAST("left")))
                {
                    xmlNodePtr ptr_left = ptr_lanes->xmlChildrenNode;//lane标签
                    xmlNodePtr ptr_left_lane;
                    while(ptr_left != NULL )
                    {
                        _xmlAttr *ptr_left_lane_tag = ptr_left->properties;
                        m_value_char = (char*)ptr_left_lane_tag->children->content;//id标签
                        m_lane.ID = charToNum<int >(m_value_char);
                        m_road.left_lane_num = m_lane.ID;//计算左侧道路条数

                        ptr_left_lane_tag = ptr_left_lane_tag->next->next;//level标签
                        m_value_char = (char*)ptr_left_lane_tag->children->content;
                        m_lane.level = charToNum<int>(m_value_char);

                        ptr_left_lane_tag = ptr_left_lane_tag->next;//uturn标签
                        m_value_char = (char*)ptr_left_lane_tag->children->content;
                        int m_uturn = charToNum<int >(m_value_char);
                        if(m_uturn == 1)
                        {
                            m_lane.is_uturn = true;
                            m_road.is_has_uturn = true;
                        } else{
                            m_lane.is_uturn = false;
                        }

                        ptr_left_lane = ptr_left->xmlChildrenNode;//link标签
                        ptr_left_lane = ptr_left_lane->next;//width标签
                        ptr_left_lane_tag = ptr_left_lane->properties;//sOffset标签

                        ptr_left_lane_tag = ptr_left_lane_tag->next;//a标签,偏移距离
                        m_value_char = (char*)ptr_left_lane_tag->children->content;
                        m_lane.offset = charToNum<float>(m_value_char);
                        m_lane.width = m_lane.offset;//偏移距离和道路宽度一致

                        ptr_left_lane_tag = ptr_left_lane_tag->next;//b标签,偏移角度
                        m_value_char = (char*)ptr_left_lane_tag->children->content;
                        m_lane.angle = charToNum<float>(m_value_char);

                        ptr_left_lane = ptr_left_lane->next;//roadMark标签
                        ptr_left_lane_tag = ptr_left_lane->properties;//sOffset标签
                        ptr_left_lane_tag = ptr_left_lane_tag->next;//指向type

                        if(!xmlStrcmp(ptr_left_lane_tag->children->content, BAD_CAST("solid")))
                        {
                            m_lane.type = 0;
                        }else if (!xmlStrcmp(ptr_left_lane_tag->children->content, BAD_CAST("broken")))
                        {
                            m_lane.type = 1;
                        }
                        m_road.lanes.push_back(m_lane);

                        ptr_left = ptr_left->next;
                    }
                }
                ptr_lanes = ptr_lanes->next;
            }

        }

        my_roads_vec_.push_back(m_road);
        pcur = pcur->next;
    }

/*****************解析junction节点中的内容********************/
    while (pcur != NULL && !xmlStrcmp(pcur->name, BAD_CAST("junction")) )
    {

        Junction m_junction;
        _xmlAttr *ptr_junction_tag = pcur->properties;
        ptr_junction_tag = ptr_junction_tag->next;//id标签
        m_value_char = (char*)ptr_junction_tag->children->content;//id标签
        m_junction.id = charToNum<int >(m_value_char);

        xmlNodePtr ptr_junction = pcur->xmlChildrenNode;//connection标签
        _xmlAttr *ptr_connection_tag;
        int num = 0;
        while(ptr_junction != NULL && !xmlStrcmp(ptr_junction->name, BAD_CAST("connection")))
        {
            ptr_connection_tag = ptr_junction->properties;//id
            ptr_connection_tag = ptr_connection_tag->next;//incomingRoad标签
            m_value_char = (char*)ptr_connection_tag->children->content;
            int incomingRoad = charToNum<int >(m_value_char);
            m_junction.incomingRoad.push_back(incomingRoad);

            ptr_connection_tag = ptr_connection_tag->next;//connectingRoad标签
            m_value_char = (char*)ptr_connection_tag->children->content;
            int connectingRoad = charToNum<int >(m_value_char);
            m_junction.connectingRoad.push_back(connectingRoad);


            ptr_junction = ptr_junction->next;
            num++;
        }
        m_junction.id_num = num;
        my_junction_vec_.push_back(m_junction);


        pcur= pcur->next;
    }

    /*****************解析connection_road_id节点中的内容********************/
    while (pcur != NULL && !xmlStrcmp(pcur->name, BAD_CAST("connection_road_id")) )
    {
        xmlNodePtr ptr_connection = pcur->xmlChildrenNode;//connection标签
        _xmlAttr *ptr_connection_tag;

        while(ptr_connection != NULL && !xmlStrcmp(ptr_connection->name, BAD_CAST("connection")))
        {
            Connection_id m_connection_id;
            ptr_connection_tag = ptr_connection->properties;//id
            m_value_char = (char*)ptr_connection_tag->children->content;
            int id = charToNum<int >(m_value_char);
            m_connection_id.id = id;
            ptr_connection_tag = ptr_connection_tag->next;//a标签
            int num = 0;

            while (ptr_connection_tag != NULL)
            {
                if(!xmlStrcmp(ptr_connection_tag->name, BAD_CAST("a"))){
                    m_value_char = (char*)ptr_connection_tag->children->content;
                    int a = charToNum<int >(m_value_char);
                    m_connection_id.connect_road_id.push_back(a);

                }else if(!xmlStrcmp(ptr_connection_tag->name, BAD_CAST("b"))){
                    m_value_char = (char*)ptr_connection_tag->children->content;
                    int b = charToNum<int >(m_value_char);
                    m_connection_id.connect_road_id.push_back(b);

                }else if(!xmlStrcmp(ptr_connection_tag->name, BAD_CAST("c"))){
                    m_value_char = (char*)ptr_connection_tag->children->content;
                    int c = charToNum<int >(m_value_char);
                    m_connection_id.connect_road_id.push_back(c);

                } else if(!xmlStrcmp(ptr_connection_tag->name, BAD_CAST("d"))){
                    m_value_char = (char*)ptr_connection_tag->children->content;
                    int d = charToNum<int >(m_value_char);
                    m_connection_id.connect_road_id.push_back(d);

                } else if(!xmlStrcmp(ptr_connection_tag->name, BAD_CAST("e"))){
                    m_value_char = (char*)ptr_connection_tag->children->content;
                    int e = charToNum<int >(m_value_char);
                    m_connection_id.connect_road_id.push_back(e);
                }
                num++;
                ptr_connection_tag = ptr_connection_tag->next;
            }

            ptr_connection = ptr_connection->next;

            m_connection_id.id_num = num;
            my_connection_id_vec_.push_back(m_connection_id);
        }
        pcur= pcur->next;
    }


/*****************释放资源********************/
    xmlFreeDoc (pdoc);
    xmlCleanupParser ();
    xmlMemoryDump ();
}


/**
 * @brief 将解析好的高精度地图文件中的世界坐标点,转化成gps的形式,并制作车道线和车道中心线
 */
void GlobalPlanner::Convert2Gps( )
{
    for(int i=0; i<my_roads_vec_.size(); ++i)
    {
        /*************判断是否为单行道*******************/
        if(my_roads_vec_[i].left_lane_num == 0 || my_roads_vec_[i].right_lane_num == 0)
        {
            my_roads_vec_[i].is_oneway = true;
            //cout<<my_roads_vec_[i].id<<"   "<<my_roads_vec_[i].road_type<<endl;
        } else{
            my_roads_vec_[i].is_oneway = false;
        }
        /*************referenceLine中的endPoint**************/
        int tmp_type = my_roads_vec_[i].referenceLine.type;
        double tmp_hdg = my_roads_vec_[i].referenceLine.hdg;
        float tmp_length = my_roads_vec_[i].referenceLine.length;
        CvPoint2D64f tmp_startPoint = my_roads_vec_[i].referenceLine.startPoint;
        double dx,dy;
        if(tmp_type == 0)               //车道为直道
        {
            dx = tmp_length * cos(tmp_hdg);
            dy = tmp_length * sin(tmp_hdg);
            my_roads_vec_[i].referenceLine.endPoint.x = tmp_startPoint.x + dx;
            my_roads_vec_[i].referenceLine.endPoint.y = tmp_startPoint.y + dy;

            /*************1m插值一个路点**************/
            int roadPoints_num = (int) tmp_length;
            CvPoint2D64f tmp_point_w;//世界坐标系
            CvPoint2D64f tmp_point_gps;//经纬度
            node tmp_node_gps;
            vector<node> tmp_referline_nodeVec;
            if(tmp_referline_nodeVec.size() > 0)
                tmp_referline_nodeVec.clear();
            for(int j=0; j<roadPoints_num; ++j)
            {
                dx = j * cos(tmp_hdg);
                dy = j * sin(tmp_hdg);
                tmp_point_w.x = tmp_startPoint.x + dx;
                tmp_point_w.y = tmp_startPoint.y + dy;
                my_roads_vec_[i].referenceLine.referLine_node_w.push_back(tmp_point_w);//id为零的车道线

                tmp_point_gps = my_gps_data_.WorldPoint2Gps(origin_gps_,tmp_point_w);//转化成gps值
                tmp_node_gps.lat = tmp_point_gps.x;
                tmp_node_gps.lng = tmp_point_gps.y;
                my_roads_vec_[i].lanes[0].laneline_node.push_back(tmp_node_gps);

                tmp_referline_nodeVec.push_back(tmp_node_gps);
            }
            /*************************构建车道线及车道中心线************************/
            int num = my_roads_vec_[i].lanes.size();
            for(int j=1; j<num; ++j)
            {
                double laneWidth = my_roads_vec_[i].lanes[j].width;
                int laneId = my_roads_vec_[i].lanes[j].ID;
                double lane_angle = my_roads_vec_[i].lanes[j].angle;//车道相对于上一车道的偏移角度
                if(laneId < 0)//右侧道路
                {
                    if(lane_angle == 0)//车道线相对于上一条道角度为零时,直接平移
                    {
                        double tmp_laneWidth = -1 * laneWidth;
                        my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneWidth);//车道线gps点

                        tmp_laneWidth = tmp_laneWidth/2;
                        my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点


                    } else             //车道线相对于上一条道角度不为零时,需要重新计算车道线
                    {
                        double tmp_laneWidth = -1 * laneWidth;
                        double tmp_laneAngle = -1 * lane_angle * 180 /PI;//转化为弧度
                        vector<node> tmp_nodeVec;//临时储存平移后的车道线,供旋转使用
                        my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,tmp_nodeVec,tmp_laneWidth);//车道线先平移再旋转
                        my_gps_data_.RotateLaneLine(tmp_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneAngle,origin_gps_);//旋转车道线

                        if(laneId == -1)//离referLine最近的车道道路中心线应该与左边的referLine车道线平行
                        {
                            if(laneWidth == 0)//处理分流车道,车道变多时,第一条车道中心点与referLine间隔半个车道
                            {
                                double tmp_next_lane_width = my_roads_vec_[i].lanes[j+1].width;
                                tmp_laneWidth = -1 * tmp_next_lane_width / 2;
                                my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);

                            } else{
                                tmp_laneWidth = tmp_laneWidth/2;
                                my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道线先平移再旋转
                            }

                        } else  //其他与右侧车道线平行,右侧的车道线向左平移半个车道
                        {
                            tmp_laneWidth = -1 * tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道线先平移再旋转
                        }

                    }

                }else if(laneId > 0)//左侧道路
                {
                    if(lane_angle == 0)//车道线相对于referLine角度为零时,直接平移
                    {
                        if(laneId == 1)//左边第一条道要相对于referLine平移,不能相对于[j-1]平移
                        {
                            double tmp_laneWidth = laneWidth;
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneWidth);//车道线gps点
                            tmp_laneWidth = tmp_laneWidth / 2;
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点

                        } else
                        {
                            double tmp_laneWidth = laneWidth;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneWidth);//车道线gps点
                            tmp_laneWidth = tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点
                        }

                    } else             //车道线相对于referLine角度不为零时,需要重新计算车道线
                    {
                        if(laneId == 1)//左边第一条道要相对于referLine平移,不能相对于[j-1]平移
                        {
                            double tmp_laneWidth = laneWidth;
                            double tmp_laneAngle = lane_angle * 180 /PI;//转化为弧度
                            vector<node> tmp_nodeVec;//临时储存平移后的车道线,供旋转使用
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,tmp_nodeVec,tmp_laneWidth);//车道线先平移再旋转
                            my_gps_data_.RotateLaneLine(tmp_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneAngle,origin_gps_);//旋转车道线

                            tmp_laneWidth = tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点

                        } else
                        {
                            double tmp_laneWidth = laneWidth;
                            double tmp_laneAngle = lane_angle * 180 /PI;//转化为弧度
                            vector<node> tmp_nodeVec;//临时储存平移后的车道线,供旋转使用
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,tmp_nodeVec,tmp_laneWidth);//车道线先平移再旋转
                            my_gps_data_.RotateLaneLine(tmp_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneAngle,origin_gps_);//旋转车道线

                            tmp_laneWidth = -1 *  tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);
                        }
                    }

                }
            }

        } else if(tmp_type == 1)        //车道为弯道
        {
            double c = my_roads_vec_[i].referenceLine.curvature;
            double hdg = tmp_hdg - PI / 2;
            double a = 2 / c * sin(tmp_length * c / 2);
            double alpha = (PI - tmp_length * c) / 2 - hdg;
            dx = -1 * a * cos(alpha);
            dy = a * sin(alpha);
            my_roads_vec_[i].referenceLine.endPoint.x = tmp_startPoint.x + dx;
            my_roads_vec_[i].referenceLine.endPoint.y = tmp_startPoint.y + dy;

            /*************1m插值一个路点**************/
            int roadPoints_num = (int) tmp_length;
            CvPoint2D64f tmp_point_w;//世界坐标系
            CvPoint2D64f tmp_point_gps;//经纬度
            node tmp_node_gps;
            vector<node> tmp_referline_nodeVec;
            if(tmp_referline_nodeVec.size() > 0)
                tmp_referline_nodeVec.clear();
            for(int j=0; j<roadPoints_num; ++j)
            {
                a = 2 / c * sin(j * c / 2);
                alpha = (PI - j * c) / 2 - hdg;
                dx = -1 * a * cos(alpha);
                dy = a * sin(alpha);
                tmp_point_w.x = tmp_startPoint.x + dx;
                tmp_point_w.y = tmp_startPoint.y + dy;
                my_roads_vec_[i].referenceLine.referLine_node_w.push_back(tmp_point_w);//id为零的车道线

                tmp_point_gps = my_gps_data_.WorldPoint2Gps(origin_gps_,tmp_point_w);//转化成gps值
                tmp_node_gps.lat = tmp_point_gps.x;
                tmp_node_gps.lng = tmp_point_gps.y;
                my_roads_vec_[i].lanes[0].laneline_node.push_back(tmp_node_gps);

                tmp_referline_nodeVec.push_back(tmp_node_gps);
            }
            /*************************构建车道线及车道中心线************************/
            int num = my_roads_vec_[i].lanes.size();
            for(int j=1; j<num; ++j)
            {
                double laneWidth = my_roads_vec_[i].lanes[j].width;
                int laneId = my_roads_vec_[i].lanes[j].ID;
                double lane_angle = my_roads_vec_[i].lanes[j].angle;//车道相对于上一车道的偏移角度
                if(laneId < 0)//右侧道路
                {
                    if(lane_angle == 0)//车道线相对于上一条道角度为零时,直接平移
                    {
                        double tmp_laneWidth = -1 * laneWidth;
                        my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneWidth);//车道线gps点

                        tmp_laneWidth = tmp_laneWidth/2;
                        my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点


                    } else             //车道线相对于上一条道角度不为零时,需要重新计算车道线
                    {
                        double tmp_laneWidth = -1 * laneWidth;
                        double tmp_laneAngle = -1 * lane_angle * 180 /PI;//转化为弧度
                        vector<node> tmp_nodeVec;//临时储存平移后的车道线,供旋转使用
                        my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,tmp_nodeVec,tmp_laneWidth);//车道线先平移再旋转
                        my_gps_data_.RotateLaneLine(tmp_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneAngle,origin_gps_);//旋转车道线

                        if(laneId == -1)//离referLine最近的车道道路中心线应该与左边的referLine车道线平行
                        {
                            if(laneWidth == 0)//处理分流车道,车道变多时,第一条车道中心点与referLine间隔半个车道
                            {
                                double tmp_next_lane_width = my_roads_vec_[i].lanes[j+1].width;
                                tmp_laneWidth = -1 * tmp_next_lane_width / 2;
                                my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);

                            } else{
                                tmp_laneWidth = tmp_laneWidth/2;
                                my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道线先平移再旋转
                            }

                        } else  //其他与右侧车道线平行,右侧的车道线向左平移半个车道
                        {
                            tmp_laneWidth = -1 * tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道线先平移再旋转
                        }


                    }

                }else if(laneId > 0)//左侧道路
                {
                    if(lane_angle == 0)//车道线相对于referLine角度为零时,直接平移
                    {
                        if(laneId == 1)//左边第一条道要相对于referLine平移,不能相对于[j-1]平移
                        {
                            double tmp_laneWidth = laneWidth;
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneWidth);//车道线gps点
                            tmp_laneWidth = tmp_laneWidth / 2;
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点

                        } else
                        {
                            double tmp_laneWidth = laneWidth;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneWidth);//车道线gps点
                            tmp_laneWidth = tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点
                        }

                    } else             //车道线相对于referLine角度不为零时,需要重新计算车道线
                    {
                        if(laneId == 1)//左边第一条道要相对于referLine平移,不能相对于[j-1]平移
                        {
                            double tmp_laneWidth = laneWidth;
                            double tmp_laneAngle = lane_angle * 180 /PI;//转化为弧度
                            vector<node> tmp_nodeVec;//临时储存平移后的车道线,供旋转使用
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,tmp_nodeVec,tmp_laneWidth);//车道线先平移再旋转
                            my_gps_data_.RotateLaneLine(tmp_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneAngle,origin_gps_);//旋转车道线

                            tmp_laneWidth = tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(tmp_referline_nodeVec,my_roads_vec_[i].lanes[j].lane_node,tmp_laneWidth);//车道中心gps点

                        } else
                        {
                            double tmp_laneWidth = laneWidth;
                            double tmp_laneAngle = lane_angle * 180 /PI;//转化为弧度
                            vector<node> tmp_nodeVec;//临时储存平移后的车道线,供旋转使用
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,tmp_nodeVec,tmp_laneWidth);//车道线先平移再旋转
                            my_gps_data_.RotateLaneLine(tmp_nodeVec,my_roads_vec_[i].lanes[j].laneline_node,tmp_laneAngle,origin_gps_);//旋转车道线

                            tmp_laneWidth = tmp_laneWidth/2;
                            my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[j-1].laneline_node,tmp_nodeVec,tmp_laneWidth);//车道线先平移再旋转
                            my_gps_data_.RotateLaneLine(tmp_nodeVec,my_roads_vec_[i].lanes[j].lane_node,tmp_laneAngle,origin_gps_);//旋转车道线
                        }
                    }

                }
            }
        }
    }
    /**********************左侧车道线gps点反转存储************************/
    for(int i = 0; i < my_roads_vec_.size(); ++i)
    {
        for(int j = 1; j < my_roads_vec_[i].lanes.size(); ++j)
        {
            if(my_roads_vec_[i].lanes[j].ID > 0)//左侧车道
            {
                int lane_line_num = my_roads_vec_[i].lanes[j].laneline_node.size();
                int half_num = lane_line_num / 2;
                for(int k = 0; k < half_num; ++k)//以车道线中心点为中心,前后对称交换元素
                {
                    node tmp_node;
                    tmp_node.lat = my_roads_vec_[i].lanes[j].laneline_node[k].lat;
                    tmp_node.lng = my_roads_vec_[i].lanes[j].laneline_node[k].lng;

                    my_roads_vec_[i].lanes[j].laneline_node[k].lat = my_roads_vec_[i].lanes[j].laneline_node[lane_line_num-1-k].lat;
                    my_roads_vec_[i].lanes[j].laneline_node[k].lng = my_roads_vec_[i].lanes[j].laneline_node[lane_line_num-1-k].lng;

                    my_roads_vec_[i].lanes[j].laneline_node[lane_line_num-1-k].lat = tmp_node.lat;
                    my_roads_vec_[i].lanes[j].laneline_node[lane_line_num-1-k].lng = tmp_node.lng;
                }

                int lane_node_num = my_roads_vec_[i].lanes[j].lane_node.size();
                half_num = lane_node_num / 2;
                for(int k = 0; k < half_num; ++k)//以车道中心点为中心,前后对称交换元素
                {
                    node tmp_node;
                    tmp_node.lat = my_roads_vec_[i].lanes[j].lane_node[k].lat;
                    tmp_node.lng = my_roads_vec_[i].lanes[j].lane_node[k].lng;

                    my_roads_vec_[i].lanes[j].lane_node[k].lat = my_roads_vec_[i].lanes[j].lane_node[lane_node_num-1-k].lat;
                    my_roads_vec_[i].lanes[j].lane_node[k].lng = my_roads_vec_[i].lanes[j].lane_node[lane_node_num-1-k].lng;

                    my_roads_vec_[i].lanes[j].lane_node[lane_node_num-1-k].lat = tmp_node.lat;
                    my_roads_vec_[i].lanes[j].lane_node[lane_node_num-1-k].lng = tmp_node.lng;
                }

            }

        }
    }


    /*******************输出txt图**********************************/


    for(int i = 0; i < my_roads_vec_.size(); ++i)
    {
        for(int j = 0; j < my_roads_vec_[i].lanes.size(); ++j)
        {
//            for(int k = 0; k < my_roads_vec_[i].lanes[j].lane_node.size(); ++k)
//            {
//                (*out_file_)<<setprecision(9)<<my_roads_vec_[i].lanes[j].lane_node[k].lng<<"  "<<my_roads_vec_[i].lanes[j].lane_node[k].lat<<endl;
//            }

//            for(int k = 0; k < my_roads_vec_[i].lanes[j].laneline_node.size(); ++k)
//            {
//                (*out_file_)<<setprecision(9)<<my_roads_vec_[i].lanes[j].laneline_node[k].lng<<"  "<<my_roads_vec_[i].lanes[j].laneline_node[k].lat<<endl;
//            }
        }

    }

}

/**
 * @brief 添加反向道路,
 */
void GlobalPlanner:: ConstructRoadMap( )
{
    /********************制作正向道路,存入my_way_vec_中*************/
    for(int i = 0; i < my_roads_vec_.size(); ++i)
    {
        Way m_way;
        /***************右侧道路*********************/
        m_way.id = my_roads_vec_[i].id;//id

        m_way.road_seg = m_way.id / 100;//road_seg

        m_way.length = my_roads_vec_[i].length;//length

        m_way.start_point.lat = my_roads_vec_[i].lanes[0].laneline_node[0].lat;//referLine的起点经纬度
        m_way.start_point.lng = my_roads_vec_[i].lanes[0].laneline_node[0].lng;

        int tmp_referline_num = my_roads_vec_[i].lanes[0].laneline_node.size();//referLine的终点经纬度
        m_way.end_point.lat = my_roads_vec_[i].lanes[0].laneline_node[tmp_referline_num-1].lat;
        m_way.end_point.lng = my_roads_vec_[i].lanes[0].laneline_node[tmp_referline_num-1].lng;

        m_way.lane_num = my_roads_vec_[i].right_lane_num;//道路数目为右边车道的数目

        m_way.cur_lane_id = my_roads_vec_[i].cur_right_lane_id;//当前道路用于轨迹拼接的道路id

        m_way.is_has_uturn = false;//uturn先初始化为假

        if(my_roads_vec_[i].road_type == 2 )//road_type为预路口时要判断一下
        {
            //右边第一条车道为实线时,该路口才为预路口
            if(my_roads_vec_[i].right_lane_num > 1 && my_roads_vec_[i].lanes[1].ID == -1 && my_roads_vec_[i].lanes[1].type == 0)
            {
                m_way.road_type = 2;
                if(my_roads_vec_[i].is_has_uturn)//有预路口时判断一下是否含有uturn车道
                    m_way.is_has_uturn = true;
            }else{
                m_way.road_type = 1;
            }

        }else{
            m_way.road_type = my_roads_vec_[i].road_type;
        }

        my_gps_data_.TrajectoryMove(my_roads_vec_[i].lanes[0].laneline_node,m_way.referLine_offset,-3.5);//将referLine向右边平移3.5m，供全局规划使用

        for(int j= 0; j < my_roads_vec_[i].lanes.size(); ++j)
        {
            if(my_roads_vec_[i].lanes[j].ID <= 0)//右侧道路
            {
                node tmp_node;
                Lane m_lane;
                m_lane.is_uturn = my_roads_vec_[i].lanes[j].is_uturn;
                m_lane.type = my_roads_vec_[i].lanes[j].type;
                m_lane.ID = -1 * my_roads_vec_[i].lanes[j].ID;
                m_lane.width = my_roads_vec_[i].lanes[j].width;
                m_lane.angle = my_roads_vec_[i].lanes[j].angle;
                for(int k = 0; k < my_roads_vec_[i].lanes[j].laneline_node.size(); ++k)//laneline_node
                {
                    tmp_node.lat = my_roads_vec_[i].lanes[j].laneline_node[k].lat;
                    tmp_node.lng = my_roads_vec_[i].lanes[j].laneline_node[k].lng;
                    m_lane.laneline_node.push_back(tmp_node);
                }

                for(int k = 0; k < my_roads_vec_[i].lanes[j].lane_node.size(); ++k)//lane_node
                {
                    tmp_node.lat = my_roads_vec_[i].lanes[j].lane_node[k].lat;
                    tmp_node.lng = my_roads_vec_[i].lanes[j].lane_node[k].lng;
                    m_lane.lane_node.push_back(tmp_node);
                }
                m_way.lanes.push_back(m_lane);
            }

        }
        my_way_vec_.push_back(m_way);

    }
    /********************制作反向道路,存入my_way_vec_中*************/
    for(int i = 0; i < my_roads_vec_.size(); ++i)
    {
        if(!my_roads_vec_[i].is_oneway)//单行道不制作反向道路
        {
            Way m_way;
            /***************左侧道路*********************/
            m_way.id = my_roads_vec_[i].id * 100 + 66;//反向道路id为原先的id后面加上66

            int tmp_road_seg = my_roads_vec_[i].id / 100 ;//road_seg
            m_way.road_seg = tmp_road_seg * 100 + 66;

            m_way.length = my_roads_vec_[i].length;//length

            int tmp_referline_num = my_roads_vec_[i].lanes[0].laneline_node.size();//referLine的起点经纬度
            m_way.start_point.lat = my_roads_vec_[i].lanes[0].laneline_node[tmp_referline_num-1].lat;
            m_way.start_point.lng = my_roads_vec_[i].lanes[0].laneline_node[tmp_referline_num-1].lng;

            m_way.end_point.lat = my_roads_vec_[i].lanes[0].laneline_node[0].lat;//referLine的终点经纬度
            m_way.end_point.lng = my_roads_vec_[i].lanes[0].laneline_node[0].lng;

            m_way.lane_num = my_roads_vec_[i].left_lane_num;//道路数目为左边车道的数目

            m_way.cur_lane_id = my_roads_vec_[i].cur_left_lane_id;//当前道路用于轨迹拼接的道路id

            m_way.is_has_uturn = false;//uturn先初始化为假

            if(my_roads_vec_[i].road_type == 2 )//road_type为预路口时要判断一下
            {
                //右边第一条车道为实线时,该路口才为正向的预路口,反向正好与正向相反
                if(my_roads_vec_[i].right_lane_num > 1 && my_roads_vec_[i].lanes[1].ID == -1 && my_roads_vec_[i].lanes[1].type == 0)
                {
                    m_way.road_type = 1;
                }else{
                    m_way.road_type = 2;
                    if(my_roads_vec_[i].is_has_uturn)//有预路口时判断一下是否含有uturn车道
                        m_way.is_has_uturn = true;
                }

            }else{
                m_way.road_type = my_roads_vec_[i].road_type;
            }

            for(int j= 0; j < my_roads_vec_[i].lanes.size(); ++j)
            {
                if(my_roads_vec_[i].lanes[j].ID == 0)//referLine反向存储
                {
                    node tmp_node;
                    Lane m_lane;
                    m_lane.is_uturn = my_roads_vec_[i].lanes[j].is_uturn;
                    m_lane.type = my_roads_vec_[i].lanes[j].type;
                    m_lane.ID = my_roads_vec_[i].lanes[j].ID;
                    m_lane.width = my_roads_vec_[i].lanes[j].width;
                    m_lane.angle = my_roads_vec_[i].lanes[j].angle;
                    for(int k = my_roads_vec_[i].lanes[j].laneline_node.size()-1; k >= 0; --k)//laneline_node
                    {
                        tmp_node.lat = my_roads_vec_[i].lanes[j].laneline_node[k].lat;
                        tmp_node.lng = my_roads_vec_[i].lanes[j].laneline_node[k].lng;
                        m_lane.laneline_node.push_back(tmp_node);
                    }

                    if(m_way.referLine_offset.size() > 0)
                        m_way.referLine_offset.clear();
                    my_gps_data_.TrajectoryMove(m_lane.laneline_node,m_way.referLine_offset,-3.5);//将referLine向右边平移3.5m，供全局规划使用

                    m_way.lanes.push_back(m_lane);
                }
                if(my_roads_vec_[i].lanes[j].ID > 0)//左侧道路
                {
                    node tmp_node;
                    Lane m_lane;
                    m_lane.is_uturn = my_roads_vec_[i].lanes[j].is_uturn;
                    m_lane.type = my_roads_vec_[i].lanes[j].type;
                    m_lane.ID = my_roads_vec_[i].lanes[j].ID;
                    m_lane.width = my_roads_vec_[i].lanes[j].width;
                    m_lane.angle = my_roads_vec_[i].lanes[j].angle;
                    for(int k = 0; k < my_roads_vec_[i].lanes[j].laneline_node.size(); ++k)//laneline_node
                    {
                        tmp_node.lat = my_roads_vec_[i].lanes[j].laneline_node[k].lat;
                        tmp_node.lng = my_roads_vec_[i].lanes[j].laneline_node[k].lng;
                        m_lane.laneline_node.push_back(tmp_node);
                    }

                    for(int k = 0; k < my_roads_vec_[i].lanes[j].lane_node.size(); ++k)//lane_node
                    {
                        tmp_node.lat = my_roads_vec_[i].lanes[j].lane_node[k].lat;
                        tmp_node.lng = my_roads_vec_[i].lanes[j].lane_node[k].lng;
                        m_lane.lane_node.push_back(tmp_node);
                    }
                    m_way.lanes.push_back(m_lane);
                }

            }
            my_way_vec_.push_back(m_way);
        }




    }


}


/**
 * @brief 创建预路口后的关联路段
 */
void GlobalPlanner:: ConstructConnectRoad()
{
    /*****************构建connecting_road***********************/
    int num = 1;
    for(int i = 0; i < my_way_vec_.size(); ++i)
    {

        if(my_way_vec_[i].road_type == 2)
        {
            for(int j = 0; j < my_connection_id_vec_.size(); ++j)
            {
                for(int k = 0; k < my_connection_id_vec_[j].connect_road_id.size(); ++k)
                {
                    if(my_way_vec_[i].id == my_connection_id_vec_[j].connect_road_id[k])//正向
                    {
                        //(*out_file_)<<my_way_vec_[i].id<<"***********"<<endl;
                        for(int r = 0; r < my_connection_id_vec_[j].connect_road_id.size(); ++r )
                        {
                            if(my_way_vec_[i].id == my_connection_id_vec_[j].connect_road_id[r])
                                continue;
                            int tmp_id = my_connection_id_vec_[j].connect_road_id[r];
                            int tmp_index = GetIdIndex(tmp_id);
                            int tmp_back_id = tmp_id * 100 + 66;
                            int tmp_back_index = GetIdIndex(tmp_back_id);
                            if(my_way_vec_[tmp_index].road_type == 1 && tmp_index != -1)
                            {
                                my_way_vec_[i].connect_way_id.push_back(tmp_id);
                                my_way_vec_[i].connect_way_id_index.push_back(make_pair(tmp_id,tmp_index));
                                //(*out_file_)<<tmp_id<<endl;
                            }else if(my_way_vec_[tmp_back_index].road_type == 1 && tmp_back_index != -1)
                            {
                                my_way_vec_[i].connect_way_id.push_back(tmp_back_id);
                                my_way_vec_[i].connect_way_id_index.push_back(make_pair(tmp_back_id,tmp_back_index));
                                //(*out_file_)<<tmp_back_id<<endl;
                            }
                        }

                    }else if(my_way_vec_[i].id == my_connection_id_vec_[j].connect_road_id[k] * 100 + 66)//反向
                    {
                        //(*out_file_)<<my_way_vec_[i].id<<"***********"<<endl;
                        for(int r = 0; r < my_connection_id_vec_[j].connect_road_id.size(); ++r )
                        {
                            if(my_way_vec_[i].id == my_connection_id_vec_[j].connect_road_id[r] * 100 + 66)
                                continue;
                            int tmp_id = my_connection_id_vec_[j].connect_road_id[r];
                            int tmp_index = GetIdIndex(tmp_id);
                            int tmp_back_id = tmp_id * 100 + 66;
                            int tmp_back_index = GetIdIndex(tmp_back_id);
                            if(my_way_vec_[tmp_index].road_type == 1 && tmp_index != -1)
                            {
                                my_way_vec_[i].connect_way_id.push_back(tmp_id);
                                my_way_vec_[i].connect_way_id_index.push_back(make_pair(tmp_id,tmp_index));
                                //(*out_file_)<<tmp_id<<endl;
                            }else if(my_way_vec_[tmp_back_index].road_type == 1 && tmp_back_index != -1)
                            {
                                my_way_vec_[i].connect_way_id.push_back(tmp_back_id);
                                my_way_vec_[i].connect_way_id_index.push_back(make_pair(tmp_back_id,tmp_back_index));
                                //(*out_file_)<<tmp_back_id<<endl;
                            }
                        }

                    }
                }
            }
        }
    }

/******************为uturn加关联道路**********************/
    for(int i = 0; i < my_way_vec_.size(); ++i)
    {
        if(my_way_vec_[i].is_has_uturn)
        {
            if(my_way_vec_[i].id < 9999)//正向道路都是四位数
            {
                int tmp_id = my_way_vec_[i].id * 100 + 66;//该条路的反向道路,作为uturn的下一条路
                int tmp_index = GetIdIndex(tmp_id);
                my_way_vec_[i].connect_way_id.push_back(tmp_id);
                my_way_vec_[i].connect_way_id_index.push_back(make_pair(tmp_id,tmp_index));
            } else                      //反向道路
            {
                int tmp_id = (my_way_vec_[i].id - 66) / 100;//该条路的反向道路,作为uturn的下一条路
                int tmp_index = GetIdIndex(tmp_id);
                my_way_vec_[i].connect_way_id.push_back(tmp_id);
                my_way_vec_[i].connect_way_id_index.push_back(make_pair(tmp_id,tmp_index));

            }
        }

    }


//    //检验正确性
//    for(int i = 0; i < my_way_vec_.size(); ++i)
//    {
//        if(my_way_vec_[i].road_type == 2)
//        {
//            (*out_file_)<<"*******  "<<my_way_vec_[i].id<<"  *********"<<endl;
//            for(int j = 0; j < my_way_vec_[i].connect_way_id.size(); ++j)
//            {
//                (*out_file_)<<my_way_vec_[i].connect_way_id[j]<<endl;
//            }
//            (*out_file_)<<endl;
//        }
//
//    }
}
/**
* @brief 获取地图的边界
*/
void GlobalPlanner:: GetMapBoundary()
{
    for(int i = 0; i < my_way_vec_.size(); ++i)
    {
        int lane_num = my_way_vec_[i].lane_num;
        int num = my_way_vec_[i].lanes[lane_num].laneline_node.size();
        double lng = my_way_vec_[i].lanes[lane_num].laneline_node[0].lng;
        double lat = my_way_vec_[i].lanes[lane_num].laneline_node[0].lat;
        //获得地图边界
        left_ = left_ > lng ? lng : left_;
        right_ = right_ < lng ? lng : right_;
        top_ = top_ > lat ? lat : top_;
        bottom_ = bottom_ < lat ? lat : bottom_;

        lng = my_way_vec_[i].lanes[lane_num].laneline_node[num-1].lng;
        lat = my_way_vec_[i].lanes[lane_num].laneline_node[num-1].lat;
        //获得地图边界
        left_ = left_ > lng ? lng : left_;
        right_ = right_ < lng ? lng : right_;
        top_ = top_ > lat ? lat : top_;
        bottom_ = bottom_ < lat ? lat : bottom_;
    }
}

/**
 * @brief 根据road的id获取在my_way_vec中的索引
 * @param id road的一个id
 * @return my_way_vec中的索引
 */
int GlobalPlanner::GetIdIndex(const int& id)
{
    for(int i = 0; i < my_way_vec_.size(); ++i)
    {
        if(my_way_vec_[i].id == id)
            return i;
    }

    return -1;  //没找到此id对应的索引
}

/**
 * @brief 创建道路之间的拓扑关系，将其存储到map_data中
 * @return true:创建成功 false:创建失败
 */
bool GlobalPlanner::ConstructGlobalMap()
{
    AstarNode tmp_node;//用于构造新的节点
    int begID = 0;
    for(int i = 0; i < my_way_vec_.size(); ++i)
    {
        begID=map_data_.size();
        //第一个节点
        tmp_node.lat = my_way_vec_[i].referLine_offset[0].lat;
        tmp_node.lng = my_way_vec_[i].referLine_offset[0].lng;
        tmp_node.disToChild[0] = my_way_vec_[i].length;
        tmp_node.road_type = my_way_vec_[i].road_type;
        tmp_node.road_seg = my_way_vec_[i].road_seg;
        tmp_node.childNum=1;//只有一个孩子
        tmp_node.child[0]=begID+1;//该孩子为其下一个节点，下一个节点还没压入
        tmp_node.id=map_data_.size();
        tmp_node.g=0.0;
        tmp_node.h=0.0;
        tmp_node.f=0.0;
        tmp_node.way_id = my_way_vec_[i].id;
        if(tmp_node.connect_way_id_index.size() > 0)
            tmp_node.connect_way_id_index.clear();
        map_data_.push_back(tmp_node);//把路的起始点放进map_data

        //第二个节点
        int num = my_way_vec_[i].referLine_offset.size();
        tmp_node.lat = my_way_vec_[i].referLine_offset[num-1].lat;
        tmp_node.lng = my_way_vec_[i].referLine_offset[num-1].lng;
        tmp_node.road_type = my_way_vec_[i].road_type;
        tmp_node.road_seg = my_way_vec_[i].road_seg;
        tmp_node.disToChild[0] = 0;//必须初始化为0，否则会用上一个值
        tmp_node.childNum=0;//必须初始化为0，否则会用上一个值
        tmp_node.id=map_data_.size();
        tmp_node.g=0.0;
        tmp_node.h=0.0;
        tmp_node.f=0.0;
        tmp_node.way_id = my_way_vec_[i].id;
        if(tmp_node.connect_way_id_index.size() > 0)
            tmp_node.connect_way_id_index.clear();
        if(my_way_vec_[i].connect_way_id_index.size() > 0)
        {
            for(int j = 0; j < my_way_vec_[i].connect_way_id_index.size(); ++j)
            {
                int tmp_way_id = my_way_vec_[i].connect_way_id_index[j].first;
                int tmp_way_id_index = my_way_vec_[i].connect_way_id_index[j].second * 2;
                tmp_node.connect_way_id_index.push_back(make_pair(tmp_way_id,tmp_way_id_index));
            }

        }
        map_data_.push_back(tmp_node);//把路的起始点放进map_data
    }

//    int num = 0;
//    for(int i = 0; i < map_data_.size(); ++i)
//    {
//        if(map_data_[i].connect_way_id_index.size() > 0)
//        {
//            for(int j = 0; j < map_data_[i].connect_way_id_index.size(); ++j)
//            {
//                cout<<map_data_[i].connect_way_id_index[j].first<<"**";
//                cout<<map_data_[map_data_[i].connect_way_id_index[j].second].way_id<<"  ";
//                cout<<map_data_[i].connect_way_id_index[j].second<<"**";
//                cout<<map_data_[map_data_[i].connect_way_id_index[j].second].id<<endl;
//
//            }
//            num++;
//            cout<<endl<<num<<endl;
//
//        }
//    }

    //构建路的终点与其他路的起点的关系
     for(int i = 0; i < map_data_.size()/2; ++i)
     {
         double lat1,lng1,lat2,lng2;

         if(map_data_[2*i].road_type == 2)//预路口道路
         {
             if(map_data_[2*i+1].connect_way_id_index.size() > 0)
             {
                 int tmp_child_num = map_data_[2*i+1].connect_way_id_index.size();
                 map_data_[2*i+1].childNum = tmp_child_num;
                 for(int j = 0; j < tmp_child_num; ++j)
                 {
                     int next_index = map_data_[2*i+1].connect_way_id_index[j].second;
                     map_data_[2*i+1].child[j] = next_index;
                     lat1 = map_data_[2*i+1].lat;
                     lng1 = map_data_[2*i+1].lng;
                     lat2 = map_data_[next_index].lat;
                     lng2 = map_data_[next_index].lng;
                     map_data_[2*i+1].disToChild[j] = my_gps_data_.GetDistance(lat1, lng1, lat2, lng2);
                 }

             }
         } else if(map_data_[2*i].road_type == 1)  //路上
         {
             int tmp_road_seg;

             for(int j = 0; j < map_data_.size()/2; ++j)
               {
                 if(i == j)
                     continue;
                 lat1 = map_data_[2*i+1].lat;
                 lng1 = map_data_[2*i+1].lng;
                 lat2 = map_data_[2*j].lat;
                 lng2 = map_data_[2*j].lng;
                 double tmp_dis = my_gps_data_.GetDistance(lat1, lng1, lat2, lng2);
                 if(map_data_[2*i+1].road_seg == map_data_[2*j].road_seg && tmp_dis < 10)//前后两条referLine的终点和起点之间距离小于10m,认为该两条referLine连接
                 {
                     map_data_[2*i+1].childNum = 1;
                     map_data_[2*i+1].child[0] = 2*j;
                     map_data_[2*i+1].disToChild[0] = 0;
                 }

             }
         }

     }
    return true;

}


/*******************************************Astar算法区****************************************************************/


/**
 * @brief 全局路径规划主接口
 * @return true:成功规划全局路径
 */
bool GlobalPlanner::searchPath()
{
    double lat = map_data_[goal_node_id_].lat;//将目标点的gps值记下来
    double lng = map_data_[goal_node_id_].lng;

    //从map_data_中找出与目标点位置相同的其他点，
    for(int i = 0; i< map_data_.size();i++)
    {
        if (map_data_[i].lat == lat  && map_data_[i].lng == lng )
        {
            bool result = solve(i);//调用astar处理主函数来判断该目标点能否找到最优路径
            if(result)
            {
                right_goal_node_id_ = i;//当找到最优路径，将此时在map_data_中的序号传给goal_node_ID,用来将way的id传到mdfPath变量中；
                return true;

            }
        }
    }
    return false;
}


/**
 * @brief astar算法主函数
 * @param tmp_goal_node_id 正确的goal_node_id
 * @return true:成功找到最优路径
 */
bool GlobalPlanner::solve(int tmp_goal_node_id)
{
    int bestID;//当前最优的ID
    AstarNode current;//用于新节点的添加
    goal_node_ = map_data_[tmp_goal_node_id];//便于后续引用
    start_node_  =  map_data_[start_node_id_];
    start_node_.g = 0.0;
    start_node_.h = my_gps_data_.GetDistance(start_node_.lat,start_node_.lng,goal_node_.lat,goal_node_.lng);
    start_node_.h = 0.0;
    start_node_.f = start_node_.g+start_node_.h;
    //设置map_data_中的起点的父节点为-2才可以
    map_data_[start_node_id_].parent_id  =  -2;
    open_list_.push_back(start_node_);
    while(open_list_.size() > 0)
    {
        current  =  open_list_[0];//将排序后（从小到大）的开启列表的第一个节点赋值给当前节点
        bestID = current.id;
        if (current.lat == goal_node_.lat && current.lng == goal_node_.lng )//判断当前节点是否指向目标节点
        {
            open_list_.clear();//清空开启列表
            close_list_.clear();//清空关闭列表
            break;
        }
        NextStep(current,goal_node_);
        close_list_.push_back(current);//将当前节点加到关闭列表
        open_list_.erase(open_list_.begin());//将上一个加入到关闭列表中的节点从开启列表中删除
        sort(open_list_.begin(),open_list_.end(),compare);//从小到大排序
    }

    if(bestID == tmp_goal_node_id)
        return true;
    else
        return false;

}
/**
 * @brief 处理当前节点的所有孩子节点
 * @param current 当前节点
 * @param goal_node 目标节点
 */
void GlobalPlanner:: NextStep(AstarNode & current, AstarNode &goal_node)
{
    for (int i = 0; i < current.childNum; i++)
    {
        checkNode(current,map_data_[current.child[i]],current.disToChild[i],goal_node);
    }
}
/**
 * @brief 处理当前节点
 * @param current 大当前节点
 * @param childNode 孩子节点
 * @param g 节点的g值
 * @param goal_node 目标节点
 */
void GlobalPlanner::checkNode( AstarNode & current, AstarNode &childNode,double g, AstarNode &goal_node)
{
    if (isContains(close_list_,childNode) != -1)
        return;
    int index;
    if ((index = isContains(open_list_,childNode)) != -1)//判断该点是否在开启列表中，如果在开启列表中则将该点在列表中的序号赋值给index
    {
        AstarNode *point = & open_list_[index];
        if (point->g > current.g + g)
        {
            point->parent_id = current.id;
            point->g = current.g+ g;
            point->f = point->g + point->h;
        }
    }
    else
    {
        childNode.g=current.g+g;
        childNode.h=my_gps_data_.GetDistance(childNode.lat,childNode.lng,goal_node.lat,goal_node.lng);
        childNode.f=childNode.g+childNode.h;
        childNode.parent_id=current.id;

        open_list_.push_back(childNode);//将该点加入开始列表
    }
}
/**
 * @brief 检查NodeList中有没有包含node
 * @param Nodelist
 * @param node
 * @return 找到该node返回该node在Nodelist中的索引值
 */

int GlobalPlanner::isContains(const vector<AstarNode> & Nodelist, AstarNode & node )
{
    for (int i = 0;i < Nodelist.size();i++)
    {
        if (Nodelist.at(i).id == node.id)
        {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 判断AstarNode的f值的大小
 * @param left
 * @param right
 * @return true:left的f值小于right的f值
 */
bool GlobalPlanner:: compare(const AstarNode& left,const AstarNode& right)
{
    return left.f < right.f; //升序排列
}

/**
* @brief 输出规划出来的最优路径在map_data_中的依次索引
*/
void GlobalPlanner::GetMdf()
{
    int cur_id;//当前ID
    cur_id=right_goal_node_id_;
    while(cur_id!=-2)
    {
        glo_path_index_.push_back(cur_id);
        cur_id=map_data_[cur_id].parent_id;
    }
    int beg=0;
    int end=glo_path_index_.size()-1;
    int temp;
    for(;beg<end;++beg,--end)
    {
        temp=glo_path_index_[beg];
        glo_path_index_[beg]=glo_path_index_[end];
        glo_path_index_[end]=temp;
    }
    int tmp_mapdata_index, tmp_way_id, tmp_wayvec_index;
    for(int i = 0; i < glo_path_index_.size(); i+=2)
    {
        tmp_mapdata_index = glo_path_index_[i];
        tmp_way_id = map_data_[tmp_mapdata_index].way_id;
        tmp_wayvec_index = GetIdIndex(tmp_way_id);
        mdf_path_.push_back(tmp_wayvec_index);
    }
    tmp_mapdata_index = glo_path_index_[glo_path_index_.size()-1];
    tmp_way_id = map_data_[tmp_mapdata_index].way_id;
    tmp_wayvec_index = GetIdIndex(tmp_way_id);
    if(mdf_path_[mdf_path_.size()-1] != tmp_wayvec_index)//如果最后一条道路没输入进去，把它输入到mdfpath中
        mdf_path_.push_back(tmp_wayvec_index);

//    mdf_path_.push_back(-1);//-1作为终点的标志

    for(int i = 0; i < mdf_path_.size(); ++i)
    {
        cout<<my_way_vec_[mdf_path_[i]].id<<endl;
    }

}

/**
  * @brief 获取下一段路的方向
  * @param back_path 前一条路
  * @param forth_path 后一条路
  * @return 1：直行 2：左转 3：右转 4:uturn
  */
int GlobalPlanner::GetDirection(vector<node> &back_path, vector<node> &forth_path)
{
    node pt1,pt2,pt3,pt4;
    CvPoint2D64f Point[4];
    int back_path_num = back_path.size();

    pt1 = back_path[back_path_num-2];
    pt2 = back_path[back_path_num-1];
    pt3 = forth_path[0];
    pt4 = forth_path[1];

    Point[0].x = pt1.lat;
    Point[0].y = pt1.lng;

    Point[1].x = pt2.lat;
    Point[1].y = pt2.lng;

    Point[2].x = pt3.lat;
    Point[2].y = pt3.lng;

    Point[3].x = pt4.lat;
    Point[3].y = pt4.lng;

    double dir1 = my_gps_data_.GetAngle(Point[1], Point[0]);
    double dir2 = my_gps_data_.GetAngle(Point[3], Point[2]);
    int fx;
    if (abs(dir1 - dir2)<45 || abs(360 - abs(dir1 - dir2))<45)
    {
        fx = 1;//直行
    }
    if (abs(dir1 - dir2 + 270)<45 || abs(90 - dir1 + dir2)<45)
    {
        fx = 2;//左转
    }
    if (abs(dir1 - dir2 + 90)<45 || abs(270 - dir1 + dir2)<45)
    {
        fx = 3;//右转
    }
    if (abs(abs(dir1 - dir2) - 180) < 18)
    {
        fx = 4;//Uturn
    }
    return fx;
}
/**
* @brief 全局轨迹拼接
*/
void GlobalPlanner::GetGlobalPath()
{
    for(int i = 0; i < mdf_path_.size()-1; ++i)
    {
        int tmp_index1 = mdf_path_[i];
        int tmp_index2 = mdf_path_[i+1];
        int next_way_heading = GetDirection(my_way_vec_[tmp_index1].referLine_offset,my_way_vec_[tmp_index2].referLine_offset);
        lead_turn_dir_vec_.push_back(next_way_heading);
        cout<<next_way_heading<<endl;
    }
    lead_turn_dir_vec_.push_back(-1);//结束标志为-1

    vector<LEAD> tmp_glo_path_vec;
    int next_way_lane_begin_index = 0;
    int tmp_road_seg = 1;
    for(int i = 0; i < lead_turn_dir_vec_.size()-1; ++i)
    {
        int next_way_heading = lead_turn_dir_vec_[i];
        int cur_way_index = mdf_path_[i];
        int next_way_index = mdf_path_[i+1];
        int cur_lane_index, next_lane_index;
        if(lead_turn_dir_vec_[i] == 1 )//当前路口直行
        {
            if(lead_turn_dir_vec_[i+1] == 1){
                cur_lane_index = my_way_vec_[cur_way_index].cur_lane_id;
            } else if(lead_turn_dir_vec_[i+1] == 3){
                cur_lane_index = my_way_vec_[cur_way_index].lane_num;
            } else if(lead_turn_dir_vec_[i+1] == 2 || lead_turn_dir_vec_[i+1] == 4){
                cur_lane_index = 1;
            }

            if(i == mdf_path_.size()-2)//如果下一段是最后一段路,靠右停车
            {
                next_lane_index = my_way_vec_[next_way_index].lane_num;
            } else
            {
                if(lead_turn_dir_vec_[i+1] == 1 && lead_turn_dir_vec_[i+2] == 1)//当前路口直行,下段路口也直行
                {
                    next_lane_index = my_way_vec_[next_way_index].cur_lane_id;
                } else if(lead_turn_dir_vec_[i+1] == 1 && lead_turn_dir_vec_[i+2] == 3)//当前路口直行,下段路口右转
                {
                    next_lane_index = my_way_vec_[next_way_index].lane_num;
                } else  if(lead_turn_dir_vec_[i+1] == 1 && (lead_turn_dir_vec_[i+2] == 2 || lead_turn_dir_vec_[i+2] == 4))  //当前路口直行,下段路口左转或uturn
                {
                    next_lane_index = 1;
                }
            }


            if(i > 0 && lead_turn_dir_vec_[i-1] != 1)//上一条路是弯道或Uturn时,从道路的第一个点开始
                next_way_lane_begin_index = 0;

            int tmp_size = my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node.size();
            if(lead_turn_dir_vec_[i+1] == 1)//下一条路是预路口,压入所有点
            {
                if(tmp_size > 24){
                    tmp_size -= 12;
                } else{
                    tmp_size = (tmp_size + 3) / 2;
                }
            }

            for (int j = next_way_lane_begin_index; j < tmp_size; ++j)//拟合部分前面的点先压入
            {
                LEAD tmp_node;
                tmp_node.lat = my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node[j].lat;
                tmp_node.lng = my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node[j].lng;
                tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                tmp_node.road_type = my_way_vec_[cur_way_index].road_type;
                tmp_node.cur_lane = cur_lane_index;
                tmp_node.road_seg_id = tmp_road_seg;//路段序号
                tmp_glo_path_vec.push_back(tmp_node);
            }
            if(my_way_vec_[next_way_index].lanes[next_lane_index].lane_node.size() > 23)
            {
                next_way_lane_begin_index = 10;
            } else{
                next_way_lane_begin_index = (my_way_vec_[next_way_index].lanes[next_lane_index].lane_node.size() - 3) / 2;
            }


            if(lead_turn_dir_vec_[i+1] == 1)
            {
                vector<node> insert_node;
                CurveFittingLaneChange(my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node,
                                       my_way_vec_[next_way_index].lanes[next_lane_index].lane_node, insert_node);

                int tmp_road_type;
                for (int j = 0; j < insert_node.size(); ++j)
                {
                    if(my_way_vec_[cur_way_index].road_type == 2 )//当前预路口,且直行,拟合的部分道路类型为路口
                    {
                        if(j < 22){
                            tmp_road_type = 2;
                        }else if(j < insert_node.size()-22){
                            tmp_road_type = 3;
                        } else{
                            tmp_road_type = my_way_vec_[next_way_index].road_type;
                        }
                    }else{
                        tmp_road_type = my_way_vec_[next_way_index].road_type;
                    }

                    LEAD tmp_node;
                    tmp_node.lat = insert_node[j].lat;
                    tmp_node.lng = insert_node[j].lng;
                    tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                    tmp_node.road_type = tmp_road_type;
                    tmp_node.cur_lane = cur_lane_index;
                    tmp_node.road_seg_id = tmp_road_seg;//路段序号
                    //(*out_file_)<<setprecision(9)<<tmp_node.lng<<"  "<<tmp_node.lat<<endl;
                    tmp_glo_path_vec.push_back(tmp_node);
                }
            }


        } else if(lead_turn_dir_vec_[i] == 2)//下个路口左转
        {
            cur_lane_index = 1;
            for(int j = 0; j < my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node.size();++j)//下个路口左转,用最左边的车道
            {
                LEAD tmp_node;
                tmp_node.lat = my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node[j].lat;
                tmp_node.lng = my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node[j].lng;
                tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                tmp_node.road_type = 2;
                tmp_node.cur_lane = cur_lane_index;
                tmp_node.road_seg_id = tmp_road_seg;//路段序号
                tmp_glo_path_vec.push_back(tmp_node);
            }
            if(i == mdf_path_.size()-2)//如果下一段是最后一段路,靠右停车
            {
                next_lane_index = my_way_vec_[next_way_index].lane_num;
            } else
            {
                int next_way_turn_dir = lead_turn_dir_vec_[i+1];
                if(next_way_turn_dir == 1)
                {
                    next_lane_index = my_way_vec_[next_way_index].cur_lane_id;
                } else if(next_way_turn_dir == 2){
                    next_lane_index = 1;//左转,最左侧道路
                } else if(next_way_turn_dir == 3){
                    next_lane_index = my_way_vec_[next_way_index].lane_num;//右转在最右侧道路
                } else if(next_way_turn_dir == 4){
                    next_lane_index = 1;//uturn在最左侧道路
                }
            }
            vector<node> insert_node;
            CurveFittingLRturn(my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node,
                               my_way_vec_[next_way_index].lanes[next_lane_index].lane_node, insert_node);

            for (int j = 0; j < insert_node.size(); ++j)
            {
                LEAD tmp_node;
                tmp_node.lat = insert_node[j].lat;
                tmp_node.lng = insert_node[j].lng;
                tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                tmp_node.road_type = 3;//此时是路上
                tmp_node.cur_lane = cur_lane_index;
                tmp_node.road_seg_id = tmp_road_seg;//路段序号
                //(*out_file_)<<setprecision(9)<<tmp_node.lng<<"  "<<tmp_node.lat<<endl;
                tmp_glo_path_vec.push_back(tmp_node);
            }
        }else if(lead_turn_dir_vec_[i] == 3)//下个路口右转
        {
            cur_lane_index = my_way_vec_[cur_way_index].lane_num;
            for(int j = 0; j < my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node.size();++j)//下个路口右转,用最右边的车道
            {
                LEAD tmp_node;
                tmp_node.lat = my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node[j].lat;
                tmp_node.lng = my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node[j].lng;
                tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                tmp_node.road_type = 2;
                tmp_node.cur_lane = cur_lane_index;
                tmp_node.road_seg_id = tmp_road_seg;//路段序号
                tmp_glo_path_vec.push_back(tmp_node);
            }
            if(i == mdf_path_.size()-2)//如果下一段是最后一段路,靠右停车
            {
                next_lane_index = my_way_vec_[next_way_index].lane_num;
            } else
            {
                int next_way_turn_dir = lead_turn_dir_vec_[i+1];
                if(next_way_turn_dir == 1)
                {
                    next_lane_index = my_way_vec_[next_way_index].cur_lane_id;
                } else if(next_way_turn_dir == 2){
                    next_lane_index = 1;//左转,最左侧道路
                } else if(next_way_turn_dir == 3){
                    next_lane_index = my_way_vec_[next_way_index].lane_num;//右转在最右侧道路
                } else if(next_way_turn_dir == 4){
                    next_lane_index = 1;//uturn在最左侧道路
                }
            }
            vector<node> insert_node;
            CurveFittingLRturn(my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node,
                               my_way_vec_[next_way_index].lanes[next_lane_index].lane_node, insert_node);

            for (int j = 0; j < insert_node.size(); ++j)
            {
                LEAD tmp_node;
                tmp_node.lat = insert_node[j].lat;
                tmp_node.lng = insert_node[j].lng;
                tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                tmp_node.road_type = 3;//此时是路上
                tmp_node.cur_lane = cur_lane_index;
                tmp_node.road_seg_id = tmp_road_seg;//路段序号
                //(*out_file_)<<setprecision(9)<<tmp_node.lng<<"  "<<tmp_node.lat<<endl;
                tmp_glo_path_vec.push_back(tmp_node);
            }
        } else if(lead_turn_dir_vec_[i+1] == 4)//下个路口uturn
        {
            cur_lane_index = 1;
            for(int j = 0; j < my_way_vec_[cur_way_index].lanes[1].lane_node.size();++j)//下个路口uturn,用最左边的车道
            {
                LEAD tmp_node;
                tmp_node.lat = my_way_vec_[cur_way_index].lanes[1].lane_node[j].lat;
                tmp_node.lng = my_way_vec_[cur_way_index].lanes[1].lane_node[j].lng;
                tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                tmp_node.road_type = 2;
                tmp_node.cur_lane = cur_lane_index;
                tmp_node.road_seg_id = tmp_road_seg;//路段序号
                tmp_glo_path_vec.push_back(tmp_node);
            }
            if(i == mdf_path_.size()-2)//如果下一段是最后一段路,靠右停车
            {
                next_lane_index = my_way_vec_[next_way_index].lane_num;
            } else
            {
                int next_way_turn_dir = lead_turn_dir_vec_[i+1];
                if(next_way_turn_dir == 1)
                {
                    next_lane_index = my_way_vec_[next_way_index].cur_lane_id;
                } else if(next_way_turn_dir == 2){
                    next_lane_index = 1;//左转,最左侧道路
                } else if(next_way_turn_dir == 3){
                    next_lane_index = my_way_vec_[next_way_index].lane_num;//右转在最右侧道路
                } else if(next_way_turn_dir == 4){
                    next_lane_index = 1;//uturn在最左侧道路
                }
            }
            vector<node> insert_node;
            CurveFittingUturn(my_way_vec_[cur_way_index].lanes[cur_lane_index].lane_node,
                              my_way_vec_[next_way_index].lanes[next_lane_index].lane_node, insert_node);

            for (int j = 0; j < insert_node.size(); ++j)
            {
                LEAD tmp_node;
                tmp_node.lat = insert_node[j].lat;
                tmp_node.lng = insert_node[j].lng;
                tmp_node.lane_num = my_way_vec_[cur_way_index].lane_num;
                tmp_node.road_type = 3;//此时是路上
                tmp_node.cur_lane = cur_lane_index;
                tmp_node.road_seg_id = tmp_road_seg;//路段序号
                //(*out_file_)<<setprecision(9)<<tmp_node.lng<<"  "<<tmp_node.lat<<endl;
                tmp_glo_path_vec.push_back(tmp_node);
            }
        }

        if(my_way_vec_[cur_way_index].road_type == 2  )//当前预路口,路段号加一
            tmp_road_seg++;
    }

    int last_way_index = mdf_path_[mdf_path_.size() - 1];//最后一条道
    int cur_lane_index = my_way_vec_[last_way_index].lane_num;//靠右停车
    int last_turn_dir = lead_turn_dir_vec_[lead_turn_dir_vec_.size()-1];
    int last_way_begin_index;
    if(last_turn_dir == 1)
    {
        last_way_begin_index = 11;//直行从第11个点开始
    } else{
        last_way_begin_index = 1;//弯道和uturn从第1个点开始
    }
    for(int i = last_way_begin_index; i < my_way_vec_[last_way_index].lanes[cur_lane_index].lane_node.size(); ++i)//最后一条道靠右道停车
    {
        LEAD tmp_node;
        tmp_node.lat = my_way_vec_[last_way_index].lanes[cur_lane_index].lane_node[i].lat;
        tmp_node.lng = my_way_vec_[last_way_index].lanes[cur_lane_index].lane_node[i].lng;
        tmp_node.lane_num = my_way_vec_[last_way_index].lane_num;
        tmp_node.road_type = my_way_vec_[last_way_index].road_type;
        tmp_node.cur_lane = cur_lane_index;
        tmp_node.road_seg_id = tmp_road_seg;//路段序号
        tmp_glo_path_vec.push_back(tmp_node);

    }

    /*************************预路口前换道用40m拟合**************************/
    LEAD tmp_node1, tmp_node2, tmp_node3;
    int min_index = 0, last_min_index = 0, pre_junction_index = 0;
    for(int i = 0; i < tmp_glo_path_vec.size()-1; ++i)
    {
        tmp_node1 = tmp_glo_path_vec[i];
        tmp_node2 = tmp_glo_path_vec[i+1];
        double distance;
        if(tmp_node1.road_type == 1 && tmp_node2.road_type == 2)
        {
            pre_junction_index = i + 1;
            for(int j = i+1; j > 0; --j)
            {
                tmp_node3 = tmp_glo_path_vec[j];
                distance = my_gps_data_.GetDistance(tmp_node1.lat,tmp_node1.lng,tmp_node3.lat,tmp_node3.lng);
                if(distance > 50)
                {
                    min_index = j;
                    break;
                }

            }
            for(int j = last_min_index; j < min_index; ++j)
            {
                tmp_node3 = tmp_glo_path_vec[j];
                glo_path_vec_.push_back(tmp_node3);
            }

            vector<node> insert_node;
            node pre_node1, pre_node2, next_node3, next_node4;
            pre_node1.lat = tmp_glo_path_vec[min_index].lat;
            pre_node1.lng = tmp_glo_path_vec[min_index].lng;
            pre_node2.lat = tmp_glo_path_vec[min_index+1].lat;
            pre_node2.lng = tmp_glo_path_vec[min_index+1].lng;

            next_node3.lat = tmp_glo_path_vec[pre_junction_index].lat;
            next_node3.lng = tmp_glo_path_vec[pre_junction_index].lng;
            next_node4.lat = tmp_glo_path_vec[pre_junction_index+1].lat;
            next_node4.lng = tmp_glo_path_vec[pre_junction_index+1].lng;
            CurveFittingPreJunction(pre_node1, pre_node2, next_node3, next_node4, insert_node);
            for (int j = 0; j < insert_node.size()-1; ++j)
            {
                LEAD tmp_node;
                tmp_node.lat = insert_node[j].lat;
                tmp_node.lng = insert_node[j].lng;
                tmp_node.lane_num = tmp_node1.lane_num;
                tmp_node.road_type = 1;//此时是路上
                tmp_node.cur_lane = tmp_node1.cur_lane;
                tmp_node.road_seg_id = tmp_node1.road_seg_id;//路段序号
                //(*out_file_)<<setprecision(9)<<tmp_node.lng<<"  "<<tmp_node.lat<<endl;
                glo_path_vec_.push_back(tmp_node);
            }

            last_min_index = i + 2;
        }
    }
    for(int i = last_min_index; i < tmp_glo_path_vec.size(); ++i)//最后一段路
    {
        tmp_node3 = tmp_glo_path_vec[i];
        glo_path_vec_.push_back(tmp_node3);
    }

    int id = 0;
    for(int i = 0; i < glo_path_vec_.size(); ++i)
    {
        (*out_file_)<<setprecision(12)<<id<<"\t"<<glo_path_vec_[i].lng<<"\t"<<glo_path_vec_[i].lat<<"\t"<<40<<"\t"<<glo_path_vec_[i].road_type<<"\t"<<0<<"\t"
                    <<glo_path_vec_[i].lane_num<<"\t"<<glo_path_vec_[i].cur_lane<<"\t"<<glo_path_vec_[i].road_seg_id<<"\t"<<0<<endl;
        id++;
    }
//    for(int i = 0; i < tmp_glo_path_vec.size(); ++i)
//    {
//        (*out_file_)<<setprecision(12)<<id<<"\t"<<tmp_glo_path_vec[i].lng<<"\t"<<tmp_glo_path_vec[i].lat<<"\t"<<40<<"\t"<<tmp_glo_path_vec[i].road_type<<"\t"<<0<<"\t"
//                    <<tmp_glo_path_vec[i].lane_num<<"\t"<<tmp_glo_path_vec[i].cur_lane<<"\t"<<tmp_glo_path_vec[i].road_seg_id<<"\t"<<0<<endl;
//        id++;
//    }

}

/**
     * @brief 换道曲线拟合
     * @param cur_way 当前道路
     * @param next_way 下一条路
     * @param insert_node 拟合后的点
     */
void GlobalPlanner::CurveFittingLaneChange(vector<node> & cur_way, vector<node> &next_way,vector<node> &insert_node)
{
    CvPoint2D64f *ins_points;
    CvPoint2D64f p[4];//通过4个点来进行贝塞尔拟合
    int insert_n;
    int cur_index1,next_index2;
    if(cur_way.size() > 23){
        cur_index1 = 11;
    } else{
        cur_index1 = (cur_way.size() - 3) / 2;
    }
    if(next_way.size() > 23){
        next_index2 = 10;
    } else{
        next_index2 = (next_way.size() - 3) / 2;
    }

    node cur_way_node1 = cur_way[cur_way.size()-cur_index1];
    node cur_way_node2 = cur_way[cur_way.size()-1];
    double cur_dis = my_gps_data_.GetDistance(cur_way_node2.lat, cur_way_node2.lng, cur_way_node1.lat, cur_way_node1.lng);

    node next_way_node1 = next_way[0];
    node next_way_node2 = next_way[next_index2];
    double next_dis = my_gps_data_.GetDistance(next_way_node1.lat, next_way_node1.lng, next_way_node2.lat, next_way_node2.lng);

    double distance = my_gps_data_.GetDistance(cur_way_node1.lat, cur_way_node1.lng, next_way_node2.lat, next_way_node2.lng);
    double stretch_len = distance / 3;
    insert_n = static_cast<int>(distance / 0.5);//每0.5m插入一个点
    ins_points=new CvPoint2D64f[insert_n];//插入的点

    node  strech_node1, strech_node2;
    strech_node1.lat = (stretch_len / cur_dis) * (cur_way_node2.lat - cur_way_node1.lat) + cur_way_node1.lat;
    strech_node1.lng = (stretch_len / cur_dis) * (cur_way_node2.lng - cur_way_node1.lng) + cur_way_node1.lng;

    strech_node2.lat = (stretch_len / next_dis) * (next_way_node1.lat - next_way_node2.lat) + next_way_node2.lat;
    strech_node2.lng = (stretch_len / next_dis) * (next_way_node1.lng - next_way_node2.lng) + next_way_node2.lng;

    p[0].x = cur_way_node1.lat;
    p[0].y = cur_way_node1.lng;

    p[1].x = strech_node1.lat;
    p[1].y = strech_node1.lng;

    p[2].x = strech_node2.lat;
    p[2].y = strech_node2.lng;

    p[3].x = next_way_node2.lat;
    p[3].y = next_way_node2.lng;

    my_gps_data_.Bezier(p,3,ins_points,insert_n);

    for(int i = 0; i < insert_n; ++i)
    {
        node tmp_node;
        tmp_node.lat = ins_points[i].x;
        tmp_node.lng = ins_points[i].y;
        insert_node.push_back(tmp_node);
    }

    delete[] ins_points;//析构
}
/**
     * @brief 左、右转曲线拟合
     * @param cur_way 当前道路
     * @param next_way 下一条路
     * @param insert_node 拟合后的点
     */
void GlobalPlanner::CurveFittingLRturn(vector<node> & cur_way, vector<node> &next_way, vector<node> &insert_node)
{
    CvPoint2D64f *ins_points;
    CvPoint2D64f p[4];//通过4个点来进行贝塞尔拟合
    int insert_n;
    node cur_way_node1 = cur_way[cur_way.size()-6];
    node cur_way_node2 = cur_way[cur_way.size()-1];
    double cur_dis = my_gps_data_.GetDistance(cur_way_node2.lat, cur_way_node2.lng, cur_way_node1.lat, cur_way_node1.lng);

    node next_way_node1 = next_way[0];
    node next_way_node2 = next_way[5];
    double next_dis = my_gps_data_.GetDistance(next_way_node1.lat, next_way_node1.lng, next_way_node2.lat, next_way_node2.lng);

    double distance = my_gps_data_.GetDistance(cur_way_node1.lat, cur_way_node1.lng, next_way_node2.lat, next_way_node2.lng);
    double stretch_len = distance / 2.3;
    insert_n = static_cast<int>(distance / 0.5);//每0.5m插入一个点
    ins_points=new CvPoint2D64f[insert_n];//插入的点


    node  strech_node1, strech_node2;
    strech_node1.lat = (stretch_len / cur_dis) * (cur_way_node2.lat - cur_way_node1.lat) + cur_way_node1.lat;
    strech_node1.lng = (stretch_len / cur_dis) * (cur_way_node2.lng - cur_way_node1.lng) + cur_way_node1.lng;

    strech_node2.lat = (stretch_len / next_dis) * (next_way_node1.lat - next_way_node2.lat) + next_way_node2.lat;
    strech_node2.lng = (stretch_len / next_dis) * (next_way_node1.lng - next_way_node2.lng) + next_way_node2.lng;

    p[0].x = cur_way_node2.lat;
    p[0].y = cur_way_node2.lng;

    p[1].x = strech_node1.lat;
    p[1].y = strech_node1.lng;

    p[2].x = strech_node2.lat;
    p[2].y = strech_node2.lng;

    p[3].x = next_way_node1.lat;
    p[3].y = next_way_node1.lng;

    my_gps_data_.Bezier(p,3,ins_points,insert_n);

    for(int i = 0; i < insert_n; ++i)
    {
        node tmp_node;
        tmp_node.lat = ins_points[i].x;
        tmp_node.lng = ins_points[i].y;
        insert_node.push_back(tmp_node);
    }

    delete[] ins_points;//析构
}
/**
    * @brief uturn曲线拟合
    * @param cur_way 当前道路
    * @param next_way 下一条路
    * @param insert_node 拟合后的点
    */
void GlobalPlanner::CurveFittingUturn(vector<node> & cur_way, vector<node> &next_way, vector<node> &insert_node)
{
    vector<node> cur_way_offset,next_way_offset;
    my_gps_data_.TrajectoryMove(cur_way,cur_way_offset,-1.7);
    my_gps_data_.TrajectoryMove(next_way,next_way_offset,-1.7);
    CvPoint2D64f *ins_points;
    CvPoint2D64f p[4];//通过4个点来进行贝塞尔拟合
    int insert_n;
    node cur_way_node1 = cur_way_offset[cur_way_offset.size()-4];
    node cur_way_node2 = cur_way_offset[cur_way_offset.size()-1];
    double cur_dis = my_gps_data_.GetDistance(cur_way_node2.lat, cur_way_node2.lng, cur_way_node1.lat, cur_way_node1.lng);

    node next_way_node1 = next_way_offset[0];
    node next_way_node2 = next_way_offset[3];
    double next_dis = my_gps_data_.GetDistance(next_way_node1.lat, next_way_node1.lng, next_way_node2.lat, next_way_node2.lng);

    //double distance = my_gps_data_.GetDistance(cur_way_node1.lat, cur_way_node1.lng, next_way_node2.lat, next_way_node2.lng);
    double stretch_len = 10;
    insert_n = 40;//插入40个点
    ins_points=new CvPoint2D64f[insert_n];//插入的点


    node  strech_node1, strech_node2;
    strech_node1.lat = (stretch_len / cur_dis) * (cur_way_node2.lat - cur_way_node1.lat) + cur_way_node2.lat;
    strech_node1.lng = (stretch_len / cur_dis) * (cur_way_node2.lng - cur_way_node1.lng) + cur_way_node2.lng;

    strech_node2.lat = (stretch_len / next_dis) * (next_way_node1.lat - next_way_node2.lat) + next_way_node1.lat;
    strech_node2.lng = (stretch_len / next_dis) * (next_way_node1.lng - next_way_node2.lng) + next_way_node1.lng;

    p[0].x = cur_way[cur_way.size()-1].lat;
    p[0].y = cur_way[cur_way.size()-1].lng;

    p[1].x = strech_node1.lat;
    p[1].y = strech_node1.lng;

    p[2].x = strech_node2.lat;
    p[2].y = strech_node2.lng;

    p[3].x = next_way[0].lat;
    p[3].y = next_way[0].lng;

    my_gps_data_.Bezier(p,3,ins_points,insert_n);

    for(int i = 0; i < insert_n; ++i)
    {
        node tmp_node;
        tmp_node.lat = ins_points[i].x;
        tmp_node.lng = ins_points[i].y;
        insert_node.push_back(tmp_node);
    }

    delete[] ins_points;//析构

}
/**
 * @brief 预路口前轨迹拟合
 * @param pre_node1 预路口前一段路的倒数第二个点
 * @param pre_node2 预路口前一段路的最后一个点
 * @param next_node3 预路口的第一个点
 * @param next_node4 预路口的第二个点
 * @param insert_node 拟合后的点
 */
void GlobalPlanner::CurveFittingPreJunction(node pre_node1, node pre_node2,node next_node3,node next_node4, vector<node> &insert_node)
{

    CvPoint2D64f *ins_points;
    CvPoint2D64f p[4];//通过4个点来进行贝塞尔拟合
    int insert_n;
    double dy = next_node4.lat - next_node3.lat;
    double dx = next_node4.lng - next_node3.lng;
    double k = dy / dx;
    if(dy > 0)
        pre_node2.lat = pre_node1.lat + sqrt((k * k) / (k * k + 1));
    else
        pre_node2.lat = pre_node1.lat- sqrt((k * k) / (k * k + 1));

    if(dx > 0)
        pre_node2.lng = pre_node1.lng + sqrt(1 / (k * k + 1));
    else
        pre_node2.lng = pre_node1.lng - sqrt(1 / (k * k + 1));

    double cur_dis = my_gps_data_.GetDistance(pre_node1.lat, pre_node1.lng, pre_node2.lat, pre_node2.lng);

    double next_dis = my_gps_data_.GetDistance(next_node3.lat, next_node3.lng, next_node4.lat, next_node4.lng);

    double distance = my_gps_data_.GetDistance(pre_node1.lat, pre_node1.lng, next_node4.lat, next_node4.lng);
    double stretch_len = distance / 3;
    insert_n = static_cast<int>(distance / 0.5);//每0.5m插入一个点
    ins_points=new CvPoint2D64f[insert_n];//插入的点

    node  strech_node1, strech_node2;
    strech_node1.lat = (stretch_len / cur_dis) * (pre_node2.lat - pre_node1.lat) + pre_node1.lat;
    strech_node1.lng = (stretch_len / cur_dis) * (pre_node2.lng - pre_node1.lng) + pre_node1.lng;

    strech_node2.lat = (stretch_len / next_dis) * (next_node3.lat - next_node4.lat) + next_node4.lat;
    strech_node2.lng = (stretch_len / next_dis) * (next_node3.lng - next_node4.lng) + next_node4.lng;

    p[0].x = pre_node1.lat;
    p[0].y = pre_node1.lng;

    p[1].x = strech_node1.lat;
    p[1].y = strech_node1.lng;

    p[2].x = strech_node2.lat;
    p[2].y = strech_node2.lng;

    p[3].x = next_node3.lat;
    p[3].y = next_node3.lng;

    my_gps_data_.Bezier(p,3,ins_points,insert_n);

    for(int i = 0; i < insert_n; ++i)
    {
        node tmp_node;
        tmp_node.lat = ins_points[i].x;
        tmp_node.lng = ins_points[i].y;
        insert_node.push_back(tmp_node);
    }

    delete[] ins_points;//析构
}












