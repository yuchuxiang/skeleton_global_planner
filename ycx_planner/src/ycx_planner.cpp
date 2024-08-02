/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <ycx_planner/ycx_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ycx_planner::YcxPlanner, nav_core::BaseGlobalPlanner)

namespace ycx_planner {

    YcxPlanner::YcxPlanner()//无参构造
            : costmap_ros_(NULL), initialized_(false){}

    YcxPlanner::YcxPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)//有参构造
            : costmap_ros_(NULL), initialized_(false){
        initialize(name, costmap_ros);
    }

    void YcxPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);

            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

            world_model_ = new base_local_planner::CostmapModel(*costmap_);

            point_duan.clear();// NOLINT
            point_fenzhi.clear();// NOLINT
            point_fenzhi_check.clear();// NOLINT
            all_key_points.clear();
            final_path.clear();

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("ycx_plan", 1);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    //we need to take the footprint of the robot into account when we calculate cost to obstacles
    double YcxPlanner::footprintCost(double x_i, double y_i, double theta_i){
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return -1.0;
        }

        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
        //if we have no footprint... do nothing
        if(footprint.size() < 3)
            return -1.0;

        //check if the footprint is legal
        double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
        return footprint_cost;
    }


    bool YcxPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

        //初始化
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }
        //打印输出
        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
        //
        plan.clear();

        {

            point_duan.clear();// NOLINT
            point_fenzhi.clear();// NOLINT
            point_fenzhi_check.clear();// NOLINT
            all_key_points.clear();
            final_path.clear();

        }

        costmap_ = costmap_ros_->getCostmap();

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                      costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        const double start_yaw = tf2::getYaw(start.pose.orientation);
        const double goal_yaw  = tf2::getYaw(goal.pose.orientation);

        //全局坐标系下的坐标
        double goal_x  = goal.pose.position.x;
        double goal_y  = goal.pose.position.y;
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;

        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;
        double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

        double target_x = goal_x;
        double target_y = goal_y;
        double target_yaw = goal_yaw;

        bool done = false;
        double scale = 1.0;
        double dScale = 0.01;

        double distance = std::sqrt(std::pow((goal_x - start_x),2) + std::pow((goal_y - start_y),2));

        if (distance<0.5){
            //采用carrot plan
            while(!done)
            {
                if(scale < 0)
                {
                    target_x = start_x;
                    target_y = start_y;
                    target_yaw = start_yaw;
                    ROS_WARN("The carrot planner could not find a valid plan for this goal");
                    break;
                }
                target_x = start_x + scale * diff_x;
                target_y = start_y + scale * diff_y;
                target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

                double footprint_cost = footprintCost(target_x, target_y, target_yaw);
                if(footprint_cost >= 0)
                {
                    done = true;
                }
                scale -=dScale;
            }

            plan.push_back(start);
            geometry_msgs::PoseStamped new_goal = goal;
            tf2::Quaternion goal_quat;
            goal_quat.setRPY(0, 0, target_yaw);

            new_goal.pose.position.x = target_x;
            new_goal.pose.position.y = target_y;

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);

            return (done);
        }


        //获取地图原点
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();

        //局部坐标系下的坐标
        double local_goal_x = goal_x - origin_x;
        double local_goal_y = goal_y - origin_y;
        double local_start_x = start_x - origin_x;
        double local_start_y = start_y - origin_y;


        //获取地图的分辨率
        double resolution_costmap = costmap_->getResolution();

        //获取局部坐标在地图中的栅格索引
        unsigned int i_goal = static_cast<unsigned int>(local_goal_x / resolution_costmap);
        unsigned int j_goal = static_cast<unsigned int>(local_goal_y / resolution_costmap);
        unsigned int i_start = static_cast<unsigned int>(local_start_x / resolution_costmap);
        unsigned int j_start = static_cast<unsigned int>(local_start_y / resolution_costmap);

        //获取地图的大小
        unsigned int costmap_width  = costmap_->getSizeInCellsX();
        unsigned int costmap_height = costmap_->getSizeInCellsY();

        //获取起点和终点
        start_p.x = i_start;
        start_p.y = j_start;

        end_p.x = i_goal;
        end_p.y = j_goal;

        ROS_INFO("AAAAAAAAAAAAAAAAA");

//        tf::Stamped<tf::Pose> start_pose;
//        tf::poseStampedMsgToTF(start, start_pose);
//        clearRobotCell(start_pose, local_start_x, local_start_y);

        costmap_->setCost(i_start, j_start, costmap_2d::FREE_SPACE);


        //将costmap_转换为cv mat，并将该mat进行二值化
        cv::Mat cv_mat(costmap_height, costmap_width, CV_8UC1);
        for (unsigned int j = 0; j < costmap_height; ++j) {
            for (unsigned int i = 0; i < costmap_width; ++i) {
                unsigned char cost = 255-costmap_->getCost(i, j);
                if (cost>15){
                    cost = 255;//黑色
                } else{
                    cost = 0;//白色
                }
//                std::cout<<"cost:"<<int(cost);
                cv_mat.at<uchar>(j, i) = cost;
            }
//            std::cout<<std::endl;
        }

        cv::Mat skeleton = cv_mat.clone();
        cv::imshow("IMGp", skeleton);
        cv::waitKey(10);

        ROS_INFO("BBBBBBBBBBBBBBBBBBB");

        thinning(cv_mat, cv_mat);

        ROS_INFO("cccccccccccccccc");

        //找出关键点
        extra_key_points(cv_mat);

        ROS_INFO("dddddddddddddddddddddd");


        check_nearst_points(cv_mat);

        ROS_INFO("eeeeeeeeeeeeeeeee");

        push_all_points();

        ROS_INFO("ffffffffffffffffffff");

        cv::Mat cw = cv_mat.clone();
        find_way(all_key_points,cw);

        if (final_path.size() == 0){
            return false;
        }

        ROS_INFO("gggggggggggggggggggg");

        std::pair<int,int> start_supply;

        start_supply.first  = i_start;
        start_supply.second = j_start;

        final_path.push_back(start_supply);

        ROS_INFO("CCCCCCCCCCCCCCC");

        //将局部路径转换为全局路径
        for (auto i = final_path.end()-1; i!= final_path.begin(); --i) {
            double target_x = (i->first+0.5)*resolution_costmap + origin_x;
            double target_y = (i->second+0.5)*resolution_costmap + origin_y;


            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map"; /// Check in which frame to publish
            pose.pose.position.x = target_x;
            pose.pose.position.y = target_y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
            plan.push_back(pose);
        }

        publishPlan(plan);

        return true;
    }

    void YcxPlanner::extra_key_points( cv::Mat& bw){
        int i,j;
        uchar *s8, *s1, *s2;    // north (pAbove)
        uchar *s7, *s0, *s3;
        uchar *s6, *s5, *s4;
        uchar *pDst;
        uchar *sAbove;//矩阵第一行的三个元素
        uchar *sCurr;
        uchar *sBelow;
        int check_1[1][2];
        //初始化行
        sAbove = NULL;
        sCurr = bw.ptr<uchar>(0);//这表示这个指针指向第一行的第一元素
        sBelow = bw.ptr<uchar>(1);//这表示这个指针指向第二行的第一元素


        for ( i = 1; i < bw.rows-1; ++i) {
            //从第二行开始，结束于倒数第二行,从上到下移动行
            sAbove=sCurr;
            sCurr=sBelow;
            sBelow=bw.ptr<uchar>(i+1);

            s1=&(sAbove[0]);
            s2=&(sAbove[1]);
            s0=&(sCurr[0]);
            s3=&(sCurr[1]);
            s5=&(sBelow[0]);
            s4=&(sBelow[1]);

            for ( j = 1; j < bw.cols-1; ++j) {
                //从左到右移动列
                s8=s1;
                s1=s2;
                s2=&(sAbove[j+1]);
                s7=s0;
                s0=s3;
                s3=&(sCurr[j+1]);
                s6=s5;
                s5=s4;
                s4=&(sBelow[j+1]);

                int s_sum= *s1 + *s2 + *s3 + *s4 + *s5 + *s6 + *s7 + *s8;
                int s_center =*s0;

                int condition_1=*s1 + *s3 + *s5 + *s7;
                int condition_1e=*s2 + *s4 + *s6 + *s8;

                int condition_2=*s2 + *s4 + *s6 + *s8;
                int condition_2e=*s1 + *s3 + *s5 + *s7;

                int condition_3=*s2 + *s4 + *s7;
                int condition_3e=*s1 + *s3 + *s5 + *s6+ *s8;

                int condition_4=*s2 + *s5 + *s8;
                int condition_4e=*s1 + *s3 + *s4 + *s6+ *s7;

                int condition_5=*s3 + *s6 + *s8;
                int condition_5e=*s1 + *s2 + *s5 + *s4+ *s7;

                int condition_6=*s1 + *s4 + *s6;
                int condition_6e=*s2 + *s3 + *s5 + *s7+ *s8;

                int condition_7=*s3 + *s5 + *s8;
                int condition_7e=*s1 + *s2 + *s4 + *s6+ *s7;

                int condition_8=*s1 + *s3 + *s6;
                int condition_8e=*s2 + *s4 + *s5 + *s7+ *s8;

                int condition_9=*s1 + *s4 + *s7;
                int condition_9e=*s2 + *s3 + *s5 + *s6+ *s8;

                int condition_10=*s2 + *s5 + *s7;
                int condition_10e=*s1 + *s3 + *s4 + *s6+ *s8;

                if (s_sum ==255 &&s_center==255){
                    std::pair<int ,int> ll;
                    ll.first=j;
                    ll.second=i;
                    point_duan.push_back(ll);
                }
                if (s_center==255&&((condition_1 == 765) || (condition_2 == 765) || (condition_3 == 765) || (condition_4 == 765) ||
                                    (condition_5 == 765) || (condition_6 == 765) || (condition_7 == 765) || (condition_8 == 765) ||
                                    (condition_9 == 765) || (condition_10 == 765))) {
                    std::pair<int, int> pp;
                    pp.first = j;
                    pp.second = i;
                    point_fenzhi.push_back(pp);
                }

            }
        }

    }

    void YcxPlanner::check_nearst_points(const cv::Mat& bw){
        float distance_start=1000;
        float distance_end=1000;

        for (int i = 0; i < bw.rows; ++i) {
            for (int j = 0; j < bw.cols; ++j) {
                if(bw.ptr<uchar>(i)[j]==255){
                    float distance_start_lingshi=abs(start_p.x-j)+ abs(start_p.y-i);
                    float distance_end_lingshi=abs(end_p.x-j)+ abs(end_p.y-i);

                    if (distance_start>distance_start_lingshi){
                        distance_start=distance_start_lingshi;
                        find_p_start.first=j;
                        find_p_start.second=i;
                    }
                    if (distance_end>distance_end_lingshi){
                        distance_end=distance_end_lingshi;
                        find_p_end.first=j;
                        find_p_end.second=i;
                    }

                }else{
                    continue;
                }

            }
        }
//        std::cout<<"sssss"<<std::endl;
//        std::cout<<find_p_start.first<<"------"<<find_p_start.second<<std::endl;
//        std::cout<<find_p_end.first<<"-----"<<find_p_end.second<<std::endl;
        cv::Point p_st,p_ed;
        p_st.x=find_p_start.first;
        p_st.y=find_p_start.second;

        all_key_points.push_back(find_p_start);

        p_ed.x=find_p_end.first;
        p_ed.y=find_p_end.second;

        all_key_points.push_back(find_p_end);

//        cv::circle(bw, p_st, 5, cvScalar(255, 100,50 ), 1, 1, 0);
//        cv::circle(bw, p_ed, 5, cvScalar(255, 100,50 ), 1, 1, 0);
//
//        cv::imshow("IMG1",bw);
    }

    void YcxPlanner::push_all_points(){
        for (int i = 0; i < point_duan.size(); ++i) {
            all_key_points.push_back(point_duan.at(i));
        }
        for (int j = 0; j < point_fenzhi_check.size(); ++j) {
            all_key_points.push_back(point_fenzhi_check.at(j));
        }
    }

    void YcxPlanner::find_way(std::deque<std::pair<int,int>> & all_points ,const cv::Mat& bw){
        int all_point_size = all_points.size();
        int start_x=all_points.front().first;
        int start_y=all_points.front().second;

        int end_x=all_points.at(1).first;
        int end_y=all_points.at(1).second;
        std::pair<int,int> end_pooo;
        end_pooo.first=end_x;
        end_pooo.second=end_y;

        std::deque<std::pair<int,int>> d1;
        std::deque<CFE> d2;
        std::deque<int> d2_l;
        std::deque<CFE_ALL> d2_all;
        std::deque<std::pair<int,int>> d3;////存储路径的队列
        CFE cfe;
        CFE_ALL cfe_all;
        std::pair<std::pair<int,int>,std::pair<int,int>> curpoint_fatherpoint;
        std::deque<std::pair<int,int>> eight_find;
        std::deque<std::pair<int,int>> eight_find_all_same_layel;

        curpoint_fatherpoint.first=all_points.front();
        curpoint_fatherpoint.second=all_points.front();
        cfe.curpoint_fatherpoint=curpoint_fatherpoint;


        d1.push_back(all_points.front());
        d2.push_back(cfe);
        d2_l.push_back(1);
        cv::Mat bw_t = bw.clone();

        bw_t.at<uchar>(d1.front().second, d1.front().first) = 0;

        int count=0;
        int label=0;
        int find_first_point_success=0;

        while (d1.size()>0){

            if(label==1)
                break;
            std::pair<int,int> father_queu;
            std::pair<int,int> son_queu;
            std::pair<std::pair<int,int>,std::pair<int,int>> c_f;
            int big_length;


            for (int i = 0; i < d1.size(); ++i) {
                int x=d1.at(i).second;
                int y=d1.at(i).first;

                father_queu.first=d1.at(i).first;
                father_queu.second=d1.at(i).second;

                eight_find= eight_neibor(x,y,bw_t);

                for (int j = 0; j < eight_find.size(); ++j) {
                    eight_find_all_same_layel.push_back(eight_find.at(j));
                    bw_t.at<uchar>(eight_find.at(j).second, eight_find.at(j).first) = 0;//访问过的节点被置为0
                    son_queu.first=eight_find.at(j).first;
                    son_queu.second=eight_find.at(j).second;
                    c_f.first=son_queu;
                    c_f.second=father_queu;
                    cfe.curpoint_fatherpoint=c_f;
                    d2.push_back(cfe);
                }
                eight_find.clear();
            }

            big_length=eight_find_all_same_layel.size();

            eight_find.clear();
            d1.clear();
            count=count+1;

            for (int k = 0; k < eight_find_all_same_layel.size(); ++k) {
                d1.push_back(eight_find_all_same_layel.at(k));
                d2_l.push_back(eight_find_all_same_layel.size());
            }
            eight_find_all_same_layel.clear();

            for (int i = 0; i < d1.size(); ++i) {
                if(d1.at(i).first==end_x && d1.at(i).second==end_y)
                    label=1;
            }

        }

        std::cout<<"the length of the path is "<<count<<std::endl;
        std::cout<<"the length of the d2 is "<<d2.size()<<std::endl;
        std::cout<<"the length of the d2_l is "<<d2_l.size()<<std::endl;

        for (int i = 0; i < d2.size(); ++i) {

            std::pair<std::pair<int,int>,std::pair<int,int>> pp;
            pp.first=d2.at(i).curpoint_fatherpoint.first;
            pp.second=d2.at(i).curpoint_fatherpoint.second;
            cfe_all.curpoint_fatherpoint_all=pp;
            cfe_all.every_length=d2_l.at(i);
            d2_all.push_back(cfe_all);
        }

        std::cout<<"the length of the d2_all is "<<d2_all.size()<<std::endl;

        while (!d2_all.empty()){
            int l=d2_all.back().every_length;
            for (int i = 0; i <l ; i++) {
                if(d2_all.back().curpoint_fatherpoint_all.first==end_pooo){
                    d3.push_back(end_pooo);
                    end_pooo=d2_all.back().curpoint_fatherpoint_all.second;
                }
                d2_all.pop_back();
            }

        }

        std::cout<<"the length of the d3 is "<<d3.size()<<std::endl;
        d3.pop_back();
        ///end points to skeletonq
        cv::Point nearest_end;
        nearest_end.x=find_p_end.first;
        nearest_end.y=find_p_end.second;
        std::deque<std::pair<int,int>> end_path_que;
        end_path_que=link_path(end_p,nearest_end);
        std::cout<<"the length of the end_path_que is "<<end_path_que.size()<<std::endl;

        if (end_path_que.size()!=0){
            for (int i = end_path_que.size()-1; i >=0; --i) {
                //std::cout<<end_path_que.at(i).first<<"xxvvvvxx"<<end_path_que.at(i).second<< std::endl;
                d3.push_front(end_path_que.at(i));
            }
        }

        ////skeleton to start points
        cv::Point nearest_start;
        nearest_start.x=find_p_start.first;
        nearest_start.y=find_p_start.second;
        std::deque<std::pair<int,int>> start_path_que;
        start_path_que=link_path(nearest_start,start_p);
        std::cout<<"the length of the start_path_que is "<<start_path_que.size()<<std::endl;

        if (start_path_que.size()!=0){
            for (const auto & i : start_path_que) {
                d3.push_back(i);
            }
        }
        std::cout<<"agregaergaergaerg"<<std::endl;

        if (!d3.empty()) {
            final_path = d3;
            std::cout << "testetestetesteteste" << std::endl;
        } else {
            std::cout << "test2 test2 test2 test2" << std::endl;
            final_path.clear();
        }

        std::cout<<"test test test test"<<std::endl;

    }

    std::deque<std::pair<int,int>> YcxPlanner::eight_neibor(int row,int col ,const cv::Mat& bw){
        //检查目标点八领域的可行点，返回其数量 和 相对位置
        std::deque<std::pair<int,int>> eight_nei;
        if(bw.at<uchar>(row-1,col)!=0){
            //std::cout<<"1 :"<<int(bw.at<uchar>(row-1,col))<<std::endl;
            std::pair<int,int> eight_1;
            eight_1.second=row-1;
            eight_1.first=col;
            eight_nei.push_back(eight_1);
        }
        if(bw.at<uchar>(row-1,col+1)!=0){
            //std::cout<<"2 :"<<int(bw.at<uchar>(row-1,col+1))<<std::endl;
            std::pair<int,int> eight_2;
            eight_2.second=row-1;
            eight_2.first=col+1;
            eight_nei.push_back(eight_2);
        }
        if(bw.at<uchar>(row,col+1)!=0){
            //std::cout<<"3 :"<<int(bw.at<uchar>(row,col+1))<<std::endl;
            std::pair<int,int> eight_3;
            eight_3.second=row;
            eight_3.first=col+1;
            eight_nei.push_back(eight_3);
        }
        if(bw.at<uchar>(row+1,col+1)!=0){
            //std::cout<<"4 :"<<int(bw.at<uchar>(row+1,col+1))<<std::endl;
            std::pair<int,int> eight_4;
            eight_4.second=row+1;
            eight_4.first=col+1;
            eight_nei.push_back(eight_4);
        }
        if(bw.at<uchar>(row+1,col)!=0){
            //std::cout<<"5 :"<<int(bw.at<uchar>(row+1,col))<<std::endl;
            std::pair<int,int> eight_5;
            eight_5.second=row+1;
            eight_5.first=col;
            eight_nei.push_back(eight_5);
        }
        if(bw.at<uchar>(row+1,col-1)!=0){
            //std::cout<<"6 :"<<int(bw.at<uchar>(row+1,col-1))<<std::endl;
            std::pair<int,int> eight_6;
            eight_6.second=row+1;
            eight_6.first=col-1;
            eight_nei.push_back(eight_6);
        }
        if(bw.at<uchar>(row,col-1)!=0){
            //std::cout<<"7 :"<<int(bw.at<uchar>(row,col-1))<<std::endl;
            std::pair<int,int> eight_7;
            eight_7.second=row;
            eight_7.first=col-1;
            eight_nei.push_back(eight_7);
        }

        if(bw.at<uchar>(row-1,col-1)!=0){
            //std::cout<<"8 :"<<int(bw.at<uchar>(row-1,col+1))<<std::endl;
            std::pair<int,int> eight_8;
            eight_8.second=row-1;
            eight_8.first=col-1;
            eight_nei.push_back(eight_8);
        }
        return eight_nei;//返回队列

    }

    std::deque<std::pair<int,int>> YcxPlanner::link_path(cv::Point p1,cv::Point p2){
        std::pair<int,int> link_2points_path;
        std::deque<std::pair<int,int>> link_path_que;
        int index_xy=0;
        float s_x=fabs(p1.x-p2.x);
        float s_y=fabs(p1.y-p2.y);
        float s_max;
        if(s_x>s_y){
            s_max=s_x;
            index_xy=1;//index==1 max=x
        }else{
            s_max=s_y;
            index_xy=1;// index==2,max=y
        }
        s_max= round(s_max);
        if(s_max==0){
            link_2points_path.first=p1.x;
            link_2points_path.second=p1.y;
            link_path_que.push_back(link_2points_path);
            return link_path_que;
        }
        if (index_xy==1){
            float inter_x,inter_y;
            inter_x=(p1.x-p2.x)/s_max;
            inter_y=(p1.y-p2.y)/s_max;
            for (int i = 0; i < s_max-1; ++i) {
                float gb_x=i*inter_x;
                float gb_y=i*inter_y;
                link_2points_path.first=int(p1.x-gb_x);
                link_2points_path.second=int(p1.y-gb_y);
                link_path_que.push_back(link_2points_path);
            }
        }
        return link_path_que;
    }

    void YcxPlanner::thinning(const cv::Mat& src, cv::Mat& dst)
    {
        dst = src.clone();
        dst /= 255;         // convert to binary image

        cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
        cv::Mat diff;

        do {
            thinningIteration(dst, 0);
            thinningIteration(dst, 1);
            cv::absdiff(dst, prev, diff);
            dst.copyTo(prev);
        }
        while (cv::countNonZero(diff) > 0);

        dst *= 255;
    }

    void YcxPlanner::thinningIteration(cv::Mat& img, int iter)
    {
        CV_Assert(img.channels() == 1);
        CV_Assert(img.depth() != sizeof(uchar));
        CV_Assert(img.rows > 3 && img.cols > 3);

        cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

        int nRows = img.rows;
        int nCols = img.cols;

        if (img.isContinuous()) {
            nCols *= nRows;
            nRows = 1;
        }

        int m, n;
        uchar *pAbove;
        uchar *pCurr;
        uchar *pBelow;
        uchar *nw, *no, *ne;    // north (pAbove)
        uchar *we, *me, *ea;
        uchar *sw, *so, *se;    // south (pBelow)

        uchar *pDst;

        // initialize row pointers
        pAbove = NULL;
        pCurr  = img.ptr<uchar>(0);
        pBelow = img.ptr<uchar>(1);

        for (m = 1; m < img.rows-1; ++m) {
            // shift the rows up by one
            pAbove = pCurr;
            pCurr  = pBelow;
            pBelow = img.ptr<uchar>(m+1);

            pDst = marker.ptr<uchar>(m);

            // initialize col pointers
            no = &(pAbove[0]);
            ne = &(pAbove[1]);
            me = &(pCurr[0]);
            ea = &(pCurr[1]);
            so = &(pBelow[0]);
            se = &(pBelow[1]);

            for (n = 1; n < img.cols-1; ++n) {
                // shift col pointers left by one (scan left to right)
                nw = no;
                no = ne;
                ne = &(pAbove[n+1]);
                we = me;
                me = ea;
                ea = &(pCurr[n+1]);
                sw = so;
                so = se;
                se = &(pBelow[n+1]);

                int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
                         (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
                         (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                         (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
                int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
                int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
                int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

//            int condition_1=*no + *ea + *so + *we;
//            int condition_2=*ne + *se + *sw + *nw;
//            int condition_3=*ne + *se + *we;
//            int condition_4=*ne + *so + *nw;
//            int condition_5=*ea + *sw + *nw;
//            int condition_6=*no + *se + *sw;
//            int condition_7=*ea + *so + *nw;
//            int condition_8=*no + *ea + *sw;
//            int condition_9=*no + *se + *we;
//            int condition_10=*ne + *so + *we;

                if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                    pDst[n] = 1;

            }
        }

        img &= ~marker;
    }

    void YcxPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        if(!path.empty())
        {
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;
        }

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for(unsigned int i=0; i < path.size(); i++){
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    void YcxPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
        if (!initialized_) {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free
        costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
    }

    //lalalala


};
