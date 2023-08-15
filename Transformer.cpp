#include <transform_hw/Transformer.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <assert.h>
#include <cmath>
#include <iostream>
using namespace std;

Transformer::Transformer(ros::NodeHandle &nh) : _nh(nh) {
  _sub = _nh.subscribe("/azure_kinect/tag_pose", 1, &Transformer::poseCallback, this);
}
Transformer::~Transformer() {}

//This method DEFINITELY takes a parameter. Can you figure out what it is?
void Transformer::poseCallback(const geometry_msgs::PoseStamped& msg) {
    
   geometry_msgs::TransformStamped _transformStamped;
   geometry_msgs::PoseStamped _msg;

    _transformStamped.header.frame_id = msg.header.frame_id;
    // _transformStamped.header.frame_id = msg.header.stamp;
    // _transformStamped.header.stamp = msg.header.stamp;
    _transformStamped.header.stamp = ros::Time::now();
    // ROS_INFO_STREAM("seq: " << msg.header.seq);
    _transformStamped.child_frame_id = "april_tag";
    _transformStamped.transform.translation.x = msg.pose.position.x;
    _transformStamped.transform.translation.y = msg.pose.position.y;
    _transformStamped.transform.translation.z = msg.pose.position.z;
    _transformStamped.header.seq = msg.header.seq;
    _transformStamped.transform.rotation.x = msg.pose.orientation.x;
    _transformStamped.transform.rotation.y = msg.pose.orientation.y;
    _transformStamped.transform.rotation.z = msg.pose.orientation.z;
    _transformStamped.transform.rotation.w = msg.pose.orientation.w;

    ROS_INFO_STREAM( "PARENT FRAME:   " << _transformStamped.header.frame_id);
    ROS_INFO_STREAM("CHILD FRAME:   " << _transformStamped.child_frame_id);
    ROS_INFO_STREAM("Translation X:     " << msg.pose.position.x);
    ROS_INFO_STREAM("Translation Y:     " << msg.pose.position.y);
    ROS_INFO_STREAM("Translation Z:     " << msg.pose.position.z);
    ROS_INFO_STREAM("Rotation X:       " << msg.pose.orientation.x);
    ROS_INFO_STREAM("Rotation Y:        " << msg.pose.orientation.y);
    ROS_INFO_STREAM("Rotation Z:        " << msg.pose.orientation.z);
    ROS_INFO_STREAM("Rotation W:        " << msg.pose.orientation.w);

    // cout << _transformStamped.header.stamp << " " << msg.header.stamp << endl;
    _br.sendTransform(_transformStamped);

    /*
        PROBLEM 2, as in the PDF:

        Take the contents of the PoseStamped message received in this callback.
        Use them to fill in a geometry_msgs::TransformStamped, and send it
            the information as a transform using TF2.
        The tutorial about using TF2 to send transforms is in the header file
            for this class.
        Call this transform's frame "april_tag".
        Its parent frame should be "azure_kinect/camera_body".
    */
   Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
   
   Eigen::Matrix3d rotation = q.toRotationMatrix();
    // cout << "rotation" << rotation << endl;
   Eigen::MatrixXd t(3,1);
   t << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z; 
    // cout << "t" << t << endl;
   Eigen::Matrix4d rt;
   
   for  (int i = 0; i < 3; i++) {
       for (int j = 0; j < 3; j++) {
           rt(i, j) = rotation(i, j);
       }
   }

   for (int i = 0; i < 3; i++) {
    int c = 3;
    rt(i, c) = t(i); 
   }

   for (int j = 0; j < 4; j++) {
       int r = 3;
       if (j <= 2) {
           rt(r, j) = 0;
       }
       else {
           rt(r, j) = 1;
       }
       
   }
   
   cout << "rt" << rt << endl;

    Eigen::MatrixXd zoffsetMat = Eigen::MatrixXd::Identity(4,4);
    
    zoffsetMat(2,3) = 1;
    
    cout << "zoffsetMat" << zoffsetMat << endl;

    Eigen::MatrixXd zoffsetResult = rt * zoffsetMat; 

    cout << "zoffsetResult" << zoffsetResult << endl;

    Eigen::Quaterniond qback (zoffsetResult.block<3,3>(0,0));
    qback.normalize();

    cout << "qback" << qback.x() << endl;

   geometry_msgs::TransformStamped _transformStamped2;
   _transformStamped2.header.stamp = ros::Time::now();
   _transformStamped2.transform.translation.x = zoffsetResult(0,3);
   _transformStamped2.transform.translation.y = zoffsetResult(1,3);
   _transformStamped2.transform.translation.z = zoffsetResult(2,3);

   _transformStamped2.transform.rotation.x = qback.x();
   _transformStamped2.transform.rotation.y = qback.y();
   _transformStamped2.transform.rotation.z = qback.z();
   _transformStamped2.transform.rotation.w = qback.w();
   
    // _transformStamped2.header.seq = msg.header.seq;

    _transformStamped2.child_frame_id = "z_offset";
    _transformStamped2.header.frame_id = "azure_kinect/camera_body";

    _br.sendTransform(_transformStamped2);

    Eigen::MatrixXd zoffsetMatflip = Eigen::MatrixXd::Identity(4,4);
    
    zoffsetMatflip(2,3) = 1;
    zoffsetMatflip(0,0) = -1; 
    zoffsetMatflip(2,2) = -1; 

    // Eigen::AngleAxisd flipVect (M_PI, Eigen::Vector3d::UnitY());

    // for  (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         zoffsetMatflip(i, j) = rotMatflip(i, j);
    //     }
    // }

    Eigen::MatrixXd zoffsetflipResult = rt * zoffsetMatflip; 

    cout << "fliped "  << zoffsetflipResult;

    Eigen::Quaterniond qback2 (zoffsetflipResult.block<3,3>(0,0));
    qback2.normalize();

   geometry_msgs::TransformStamped _transformStamped3;
    _transformStamped3.header.stamp = ros::Time::now();
   _transformStamped3.transform.translation.x = zoffsetflipResult(0,3);
   _transformStamped3.transform.translation.y = zoffsetflipResult(1,3);
   _transformStamped3.transform.translation.z = zoffsetflipResult(2,3);

   _transformStamped3.transform.rotation.x = qback2.x();
   _transformStamped3.transform.rotation.y = qback2.y();
   _transformStamped3.transform.rotation.z = qback2.z();
   _transformStamped3.transform.rotation.w = qback2.w();
   
    _transformStamped3.child_frame_id = "z_offset_flip";
    _transformStamped3.header.frame_id = "azure_kinect/camera_body";

    cout << "_transformStamped3.transform.rotation.x" << _transformStamped3.transform.rotation.x; 
    _br.sendTransform(_transformStamped3);
    /*
        PROBLEM 3, as in the PDF:

        Publish a transform that is 1 meter (1.0) in front of "april_tag"
            and in the same orientation (all axes facing the same direction).
            This means 1 meter in front of the face of the marker, as in
            sticking out in front of the marker along its z axis.    

        1) Remember that transformations form chains from the "proximal" frame
            to the "distal" frame. In this case, the "proximal" frame is the
            camera "azure_kinect/camera_body". The "distal" frame is the point
            one meter in front of the marker .
        2) Use Eigen to build your chain of transformations:
            A) Chains of transformations are covered in
                Lec_Spatial_Transformations.pdf
            B) Eigen is covered in Lec_Eigen.pdf
            C) The mathematics are covered in Lec_Transformation_Mat.pdf
        3) The basic version of how to do this is:
            A) Use Eigen::Quaterniond to get the rotation matrix from the
                quaternion in geometry_msgs::PoseStamped that is received by
                this callback.
            B) Build a rigid transformation representing the transform between
                the camera and the marker. The rigid transformation is as in
                Lec_Transformation_Math. It is a 4x4 matrix (Eigen::MatrixXd).
                The upper-left 3x3 submatrix is the rotation (which can be)
                obtained from the Quaterniond that you just built using
                Eigen::Quaterniond::toRotationMatrix() The right column of the
                matrix is the translation, which can be obtained directly from
                the geometry_msgs::PoseStamped that is received by this
                callback.
                The matrix that you have just built is the rigid transformation
                between the camera and the marker.
            C) Build a rigid transformation representing the concept of "one
                meter in front along the z-axis." This transformation has an
                identity rotation component. It has 1.0 in the z translational
                component. Hint: Start by using Eigen::MatrixXd::Identity(4,4).
            D) Multiply the rigid transformation from the camera to the marker,
                and the one from the marker to the offset.
            E) Populate an Eigen::Quaterniond to figure out the quaternion 
                representing the orientation of the offset point.
                You can do this by initializing a new Eigen::Quaterniond with
                the submatrix from the 4x4 rigid transformation (the result)
                of the multiplication in Step D.
                This is a little tricky, so I'll show you what my code looks
                like here:
                Eigen::Quaterniond offsetQ(offsetOutMat.block<3,3>(0,0));
                Hint: Since the offset is facing the same direction as the
                marker, there *is* another way.
            F) Make a geometry_msgs::TransformStamped, pack it with the 
                transform to the offset. Send it.
	        G) HINT: You will have problems with your quaternion after you
		        send it if it is not normalized. So, normalize it before
                putting it into your TransformStamped. To do this, use
                Eigen::Quaterniond::normalize(). In my code, it looks like
                offsetQ.normalize();
    */

    /*
        PROBLEM 4, as in the PDF:

        Publish a transform that is 1 meter (1.0) in front of "april_tag"
            but is rotated 180 degrees about its Y axis.
    
        How to do this:
        1) Do all of the steps from PROBLEM 3.
        2) Add a final rotation at the end about the Y axis.
        3) HINT: Use Eigen::AngleAxisd to achieve this, with M_PI and
            Eigen::Vector3d::UnitY()
    */
};
