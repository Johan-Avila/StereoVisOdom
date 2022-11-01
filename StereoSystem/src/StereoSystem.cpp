#include "utility.h"

class StereoVisOdom
{
public:
    ParamServer parSer;

    ros::Subscriber sub_img0;
    ros::Subscriber sub_img1;

    ros::Publisher pub_odom;

    ros::Publisher pub_img1_flag;
    ros::Publisher pub_img0_flag;
    ros::Publisher pub_odom_flag;

    bool state_img0;
    bool state_img1;

    std_msgs::Int8 pub_flag_1;
    std_msgs::Int8 pub_flag_0;

    cv::Mat mp0;
    cv::Mat mp1;

    cv::Mat mp0_k;
    cv::Mat mp0_r;
    cv::Mat mp0_t;

    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> bf;

    int dsp_max_u;
    int dsp_min_u;
    int dsp_max_v;

    bool pnp_useExtrinsicGuess;
    int pnp_iterationsCount;
    float pnp_reprojectionError;
    double pnp_confidence;
    int pnp_flags;

    cv::Mat relative_motion_k;
    cv::Mat absolute_motion_k;
    cv::Mat absolute_motion_k_1;

    string motion_poses;

    bool tracking_flag;

    vector<cv::KeyPoint> kp0_k;
    vector<cv::KeyPoint> kp1_k;

    vector<cv::KeyPoint> kp0_k_1;
    vector<cv::KeyPoint> kp1_k_1;

    cv::Mat ds0_k;
    cv::Mat ds1_k;

    cv::Mat ds0_k_1;

    cv::Mat pts0_2D_k;
    cv::Mat pts0_2D_k_1;
    cv::Mat pts1_2D_k_1;
    cv::Mat pts_3D_k_1;

    cv_bridge::CvImagePtr img0_k;
    vector<cv::KeyPoint> good_kp0_k;

    StereoVisOdom()
    {
        this->state_img0 = false;
        this->state_img1 = false;

        this->pub_flag_1.data = 1;
        this->pub_flag_0.data = 0;

        this->mp0 = this->parSer.get_calib_matrixProjection("mp0");
        this->mp1 = this->parSer.get_calib_matrixProjection("mp1");

        cv::decomposeProjectionMatrix(this->mp0, this->mp0_k, this->mp0_r, this->mp0_t);

        this->orb = this->parSer.create_orb();
        this->bf = this->parSer.create_bf();

        this->dsp_max_u = this->parSer.get_fil_StereoGeometry("max_u");
        this->dsp_min_u = this->parSer.get_fil_StereoGeometry("min_u");
        this->dsp_max_v = this->parSer.get_fil_StereoGeometry("max_v");

        this->pnp_useExtrinsicGuess = this->parSer.pnp_useExtrinsicGuess;
        this->pnp_iterationsCount = this->parSer.pnp_iterationsCount;
        this->pnp_reprojectionError = this->parSer.pnp_reprojectionError;
        this->pnp_confidence = this->parSer.pnp_confidence;
        this->pnp_flags = this->parSer.pnp_flags;

        this->relative_motion_k = cv::Mat::eye(4, 4, cv::DataType<double>::type);
        this->absolute_motion_k = cv::Mat::eye(4, 4, cv::DataType<double>::type);
        this->absolute_motion_k_1 = cv::Mat::eye(4, 4, cv::DataType<double>::type);

        this->tracking_flag = false;
    }

    void setting_subscriber()
    {
        ros::NodeHandle nh;
        string topics_img0 = this->parSer.get_topic("img0");
        string topics_img1 = this->parSer.get_topic("img1");

        this->sub_img0 = nh.subscribe<sensor_msgs::Image>(
            topics_img0.c_str(),
            100,
            &StereoVisOdom::detect_corners_img0,
            this,
            ros::TransportHints().tcpNoDelay());

        this->sub_img1 = nh.subscribe<sensor_msgs::Image>(
            topics_img1.c_str(),
            100,
            &StereoVisOdom::detect_corners_img1,
            this,
            ros::TransportHints().tcpNoDelay());
    }

    void setting_publisher()
    {
        ros::NodeHandle nh;

        this->pub_odom = nh.advertise<nav_msgs::Odometry>(
            "/stereo_vis_odom/odom",
            100);

        this->pub_img0_flag = nh.advertise<std_msgs::Int8>(
            "/stereo_vis_odom/img0_flag",
            1);

        this->pub_img1_flag = nh.advertise<std_msgs::Int8>(
            "/stereo_vis_odom/img1_flag",
            1);

        this->pub_odom_flag = nh.advertise<std_msgs::Int8>(
            "/stereo_vis_odom/odom_flag",
            1);
    }

    void publish_odom()
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = this->absolute_motion_k.at<double>(0, 3);
        odom.pose.pose.position.y = this->absolute_motion_k.at<double>(1, 3);
        odom.pose.pose.position.z = this->absolute_motion_k.at<double>(2, 3);

        this->pub_odom.publish(odom);
    }

    void detect_corners_img0(const sensor_msgs::Image::ConstPtr &img0_ros)
    {
        this->pub_img0_flag.publish(this->pub_flag_1);
        cv_bridge::CvImagePtr img0_k = cv_bridge::toCvCopy(img0_ros);

        this->orb->detectAndCompute(
            img0_k->image,
            cv::Mat(),
            this->kp0_k,
            this->ds0_k);

        this->img0_k = img0_k;
        this->state_img0 = true;
        this->pub_img0_flag.publish(this->pub_flag_0);
    }

    void detect_corners_img1(const sensor_msgs::Image::ConstPtr &img1_ros)
    {
        this->pub_img1_flag.publish(this->pub_flag_1);
        cv_bridge::CvImagePtr img1_k = cv_bridge::toCvCopy(img1_ros);

        this->orb->detectAndCompute(
            img1_k->image,
            cv::Mat(),
            this->kp1_k,
            this->ds1_k);

        this->state_img1 = true;
        this->pub_img1_flag.publish(this->pub_flag_0);
    }

    void reset_state_imgs()
    {
        this->state_img0 = false;
        this->state_img1 = false;
    }

    void matcher_StereoFrame()
    {
        vector<cv::KeyPoint> kp0_k;
        vector<cv::KeyPoint> kp1_k;

        cv::KeyPoint kp0_tmp;
        cv::KeyPoint kp1_tmp;

        cv::Mat ds0_k;

        cv::Mat ds0_tmp;

        int dsp_u_tmp;
        int dsp_v_tmp;

        vector<cv::DMatch> maches_coner;
        this->bf->match(this->ds1_k, this->ds0_k, maches_coner, cv::Mat());

        for (size_t i = 0; i < maches_coner.size(); i++)
        {
            kp0_tmp = this->kp0_k[maches_coner[i].trainIdx];
            kp1_tmp = this->kp1_k[maches_coner[i].queryIdx];

            ds0_tmp = this->ds0_k.row(maches_coner[i].trainIdx);

            dsp_u_tmp = kp0_tmp.pt.x - kp1_tmp.pt.x;
            dsp_v_tmp = abs(kp0_tmp.pt.y - kp1_tmp.pt.y);

            // if (this->dsp_min_u <= dsp_u_tmp &&
            //     dsp_u_tmp <= this->dsp_max_u &&
            //     dsp_v_tmp <= this->dsp_max_v)
            // {
            kp0_k.push_back(kp0_tmp);
            kp1_k.push_back(kp1_tmp);

            ds0_k.push_back(ds0_tmp);
            // }
        }

        this->kp0_k = kp0_k;
        this->kp1_k = kp1_k;

        this->ds0_k = ds0_k;
    }

    void matcher_poses()
    {
        vector<cv::KeyPoint> good_kp0_k;

        cv::Mat pts0_2D_k;
        cv::Mat pts0_2D_k_1;
        cv::Mat pts1_2D_k_1;

        cv::KeyPoint kp0_k_tmp;
        cv::KeyPoint kp0_k_1_tmp;
        cv::KeyPoint kp1_k_1_tmp;

        cv::Mat pts0_2D_k_tmp(1, 2, cv::DataType<double>::type);
        cv::Mat pts0_2D_k_1_tmp(1, 2, cv::DataType<double>::type);
        cv::Mat pts1_2D_k_1_tmp(1, 2, cv::DataType<double>::type);

        vector<cv::DMatch> maches_poses;
        this->bf->match(this->ds0_k_1, this->ds0_k, maches_poses, cv::Mat());

        for (size_t i = 0; i < maches_poses.size(); i++)
        {
            kp0_k_tmp = this->kp0_k[maches_poses[i].trainIdx];
            kp0_k_1_tmp = this->kp0_k_1[maches_poses[i].queryIdx];
            kp1_k_1_tmp = this->kp1_k_1[maches_poses[i].queryIdx];

            pts0_2D_k_tmp.at<double>(0, 0) = kp0_k_tmp.pt.x;
            pts0_2D_k_tmp.at<double>(0, 1) = kp0_k_tmp.pt.y;

            pts0_2D_k_1_tmp.at<double>(0, 0) = kp0_k_1_tmp.pt.x;
            pts0_2D_k_1_tmp.at<double>(0, 1) = kp0_k_1_tmp.pt.y;

            pts1_2D_k_1_tmp.at<double>(0, 0) = kp1_k_1_tmp.pt.x;
            pts1_2D_k_1_tmp.at<double>(0, 1) = kp1_k_1_tmp.pt.y;

            good_kp0_k.push_back(this->kp0_k[maches_poses[i].trainIdx]);

            pts0_2D_k.push_back(pts0_2D_k_tmp);
            pts0_2D_k_1.push_back(pts0_2D_k_1_tmp);
            pts1_2D_k_1.push_back(pts1_2D_k_1_tmp);
        }

        this->good_kp0_k = good_kp0_k;

        this->pts0_2D_k = pts0_2D_k;
        this->pts0_2D_k_1 = pts0_2D_k_1;
        this->pts1_2D_k_1 = pts1_2D_k_1;
    }

    void triangulation_pts_3D_k_1()
    {
        cv::Mat pts_4D_k_1;
        cv::Mat pts_3D_k_1;

        cv::Mat pts0_2D_k_1;
        cv::Mat pts1_2D_k_1;

        double x_tmp;
        double y_tmp;
        double z_tmp;
        double w_tmp;

        cv::Mat pts_3D_k_1_tmp(1, 3, cv::DataType<double>::type);

        pts0_2D_k_1 = this->pts0_2D_k_1.t();
        pts1_2D_k_1 = this->pts1_2D_k_1.t();

        cv::triangulatePoints(
            this->mp0,
            this->mp1,
            pts0_2D_k_1,
            pts1_2D_k_1,
            pts_4D_k_1);

        for (int cols_idx = 0; cols_idx < pts_4D_k_1.cols; cols_idx++)
        {
            x_tmp = pts_4D_k_1.at<double>(0, cols_idx);
            y_tmp = pts_4D_k_1.at<double>(1, cols_idx);
            z_tmp = pts_4D_k_1.at<double>(2, cols_idx);
            w_tmp = pts_4D_k_1.at<double>(3, cols_idx);

            x_tmp = x_tmp / w_tmp;
            y_tmp = y_tmp / w_tmp;
            z_tmp = z_tmp / w_tmp;
            w_tmp = w_tmp / w_tmp;

            pts_3D_k_1_tmp.at<double>(0, 0) = x_tmp;
            pts_3D_k_1_tmp.at<double>(0, 1) = y_tmp;
            pts_3D_k_1_tmp.at<double>(0, 2) = z_tmp;

            pts_3D_k_1.push_back(pts_3D_k_1_tmp);
        }

        this->pts_3D_k_1 = pts_3D_k_1;
    }

    void calculation_relative_Motion()
    {
        cv::Mat rvec;
        cv::Mat tvec;
        cv::Mat OutputArray;
        cv::Mat rotationMatrix;
        cv::Mat relative_motion_k;

        cv::solvePnPRansac(
            this->pts_3D_k_1, this->pts0_2D_k, this->mp0_k, cv::Mat(), rvec, tvec,
            this->pnp_useExtrinsicGuess,
            this->pnp_iterationsCount,
            this->pnp_reprojectionError,
            this->pnp_confidence,
            OutputArray,
            this->pnp_flags);

        cv::Rodrigues(rvec, rotationMatrix);

        relative_motion_k = cv::Mat::eye(4, 4, cv::DataType<double>::type);

        relative_motion_k.at<double>(0, 0) = rotationMatrix.at<double>(0, 0);
        relative_motion_k.at<double>(0, 1) = rotationMatrix.at<double>(0, 1);
        relative_motion_k.at<double>(0, 2) = rotationMatrix.at<double>(0, 2);

        relative_motion_k.at<double>(1, 0) = rotationMatrix.at<double>(1, 0);
        relative_motion_k.at<double>(1, 1) = rotationMatrix.at<double>(1, 1);
        relative_motion_k.at<double>(1, 2) = rotationMatrix.at<double>(1, 2);

        relative_motion_k.at<double>(2, 0) = rotationMatrix.at<double>(2, 0);
        relative_motion_k.at<double>(2, 1) = rotationMatrix.at<double>(2, 1);
        relative_motion_k.at<double>(2, 2) = rotationMatrix.at<double>(2, 2);

        relative_motion_k.at<double>(0, 3) = tvec.at<double>(0, 0);
        relative_motion_k.at<double>(1, 3) = tvec.at<double>(1, 0);
        relative_motion_k.at<double>(2, 3) = tvec.at<double>(2, 0);

        this->relative_motion_k = relative_motion_k.inv();
    }

    void calculation_absolute_Motion()
    {
        this->absolute_motion_k = this->absolute_motion_k_1 * this->relative_motion_k;
        this->publish_odom();
    }

    void pose_tracking()
    {
        this->kp0_k_1 = this->kp0_k;
        this->kp1_k_1 = this->kp1_k;

        this->ds0_k_1 = this->ds0_k;

        this->absolute_motion_k_1 = this->absolute_motion_k;

        this->tracking_flag = true;

        string motion_pose;

        motion_pose.append(to_string(this->absolute_motion_k.at<double>(0, 0)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(0, 1)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(0, 2)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(0, 3)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(1, 0)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(1, 1)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(1, 2)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(1, 3)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(2, 0)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(2, 1)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(2, 2)));
        motion_pose.append(" ");
        motion_pose.append(to_string(this->absolute_motion_k.at<double>(2, 3)));
        motion_pose.append("\n");

        this->motion_poses.append(motion_pose);
    }

    void save_route()
    {
        // cout << "\n"
        //      << "save_route"
        //      << "\n"
        //      << endl;

        // string pwd_file;
        // // pwd_file.append("/home/sirui/vo_ws/src/StereoVisOdom/results/route/route_v2.txt");
        // pwd_file.append("/home/johanp/vo_ws/src/Info-StereoVisOdom/StereoSystem/route/route_v2.txt");
        // // pwd_file.append("/home/sirui/vo_ws/src/StereoVisOdom/results/route/route_v2.txt");
        // //  pwd_file.append("/home/avila/vo_ws/src/StereoVisOdom/results/route/route_v2.txt");
        // ofstream file;
        // file.open(pwd_file);
        // file << this->motion_poses;
        // file.close();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stereo_vis_odom");

    StereoVisOdom stereo_vis_odom;
    stereo_vis_odom.setting_subscriber();
    stereo_vis_odom.setting_publisher();

    ROS_INFO("\033[1;32m-> StereoSystem.\033[0m");
    while (ros::ok())
    {
        if (stereo_vis_odom.state_img0 && stereo_vis_odom.state_img1)
        {
            stereo_vis_odom.pub_odom_flag.publish(stereo_vis_odom.pub_flag_1);
            stereo_vis_odom.reset_state_imgs();
            stereo_vis_odom.matcher_StereoFrame();

            if (!stereo_vis_odom.tracking_flag)
            {
                stereo_vis_odom.pose_tracking();
                stereo_vis_odom.pub_odom_flag.publish(stereo_vis_odom.pub_flag_0);
            }

            stereo_vis_odom.matcher_poses();
            stereo_vis_odom.triangulation_pts_3D_k_1();
            stereo_vis_odom.calculation_relative_Motion();
            stereo_vis_odom.calculation_absolute_Motion();
            stereo_vis_odom.pose_tracking();
            stereo_vis_odom.pub_odom_flag.publish(stereo_vis_odom.pub_flag_0);
        }
        ros::spinOnce();
    }

    stereo_vis_odom.save_route();
    return 0;
}