// To receive the image and inspec the cube sdf dsf
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>  
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32MultiArray.h>

#include "polled_camera/GetPolledImage.h"

#include "rubiks_inspect/OcvImage.h" 

#include "cv_bridge/CvBridge.h"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#include <image_transport/camera_subscriber.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rubiks_graph/MoveToStateAction.h>

#include "rubiks_solver/SolveService.h"


struct ImageCallback {
	typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> Synchronizer;

	ros::NodeHandle node;

	OcvImage < RGBPixel<unsigned char > > image;
//	OcvImage < unsigned char  > image;

	message_filters::Subscriber < sensor_msgs::Image > imageSub;
	message_filters::Subscriber < sensor_msgs::CameraInfo > infoSub;
	Synchronizer sync;


	sensor_msgs::CvBridge cvbridge;
	image_geometry::PinholeCameraModel pinhole;

	roslib::Header image_header;

	ImageCallback(ros::NodeHandle &masternode, std::string image_name, std::string info_name)  :
		node(masternode), image(10, 10, IPL_DEPTH_8U, 3), imageSub(node,image_name,1), infoSub(node,info_name,1) ,
		sync(imageSub, infoSub, 10)	{

		sync.registerCallback(boost::bind(&ImageCallback::imageCallback, this, _1, _2));

	}

	void imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
	{
		// Get the info into the pinhole and the image into the image...
		pinhole.fromCameraInfo(cam_info);
		image.resize(image_msg->width,image_msg->height);
		memcpy(image.iplimage->imageData,&(image_msg->data[0]),sizeof(char) * image_msg->step * image_msg->height);
		image_header = image_msg->header;

		// and play with the image...
		processImage();
	}


	virtual void processImage() {};
};

struct RubiksImageCallback : ImageCallback {
	tf::TransformListener transforms;
	tf::StampedTransform transform;

	enum Face {
		F=0,U=1,B=2,L=3,R=4,D=5
	}  ;

	Face inspect_face;

	bool shot;



	cv::Mat face_pixels;

	cv::Mat cubie_colours;
	image_transport::Publisher image_publisher;
	image_transport::ImageTransport it;

	cv::Mat template_image;

	float centres[6*3];



	RubiksImageCallback(ros::NodeHandle &masternode)  : ImageCallback(masternode, "/prosilica_req/image_rect_color", "/prosilica_req/camera_info"),
//	RubiksImageCallback(ros::NodeHandle &masternode)  : ImageCallback(masternode, "/wide_stereo/right/image_rect_color", "/wide_stereo/right/camera_info"),
			face_pixels(6, 18, CV_32FC1),
			cubie_colours(54,1, CV_32FC3),
			it(masternode) {
		shot = false;
		// Face poses contains the 2D image positions of the 9 cubies for each face
		// row order is contains the F,U,B,L,R,D


		// TODO: Will set this to load from a file later...
		float FaceF[] = {1038,923,1207,932,1377,940,1040,1090,1210,1094,1377,1097,1034,1247,1210,1262,1375,1260};//
//		float FaceF[] = {1278,954,1395-1167};
		float FaceR[] = {1491,1350,1140,1352,786,1368,1463,1007,1124,1029,762,1029,1444,636,1098,653,742,668};//
//		float FaceR[] = {1032,1479, 1038-813};
		float FaceL[] = {1431,1224,1142,1198,860,1166,1437,938,1157,916,870,894,866,612,1164,621,1457,649};//
//		float FaceL[] = {1170,1509,1173-954};
		float FaceU[] = {757,833,1177,797,1580,788,773,1227,1179,1205,1580,1170,784,1626,1185,1620,1585,1585};//
//		float FaceU[] = {1308,1401,1575-1308};
		float FaceD[] = {898,764,1382,734,1834,745,889,1255,1369,1230,1850,1253,868,1707,1357,1714,1825,1725};//
//		float FaceD[] = {981,1174,1248-975};
		float FaceB[] = {1123,552,1365,542,1614,535,1125,797,1372,789,1612,781,1135,1036,1387,1028,1622,1018}; //
//		float FaceB[] = {1071,1010,1299-1077};

		// The standard distance poses
		calculateFacePoints(FaceF,1005,690,1263,936);
		calculateFacePoints(FaceB,1059,828,1284,1050);
		calculateFacePoints(FaceU,1146,720,1446,1020);
		calculateFacePoints(FaceR,879,996,1074,1194);
		rotateFacePoints(FaceR,2);
		calculateFacePoints(FaceL,945,960,1134,1158);
		rotateFacePoints(FaceL,2);
		calculateFacePoints(FaceD,864,1404,1158,1734);
		rotateFacePoints(FaceD,3);

		// The ultra close blind robot poses
//		calculateFacePoints(FaceF,1038,923,1375,1260);
//		calculateFacePoints(FaceB,1123,552,1622,1018);
//		calculateFacePoints(FaceU,757,833,1585,1585);
//
//		calculateFacePoints(FaceR,738,660,1494,1337);
//		rotateFacePoints(FaceR,2);
//		calculateFacePoints(FaceL,866,608,1435,1220);
//		rotateFacePoints(FaceL,2);
//		calculateFacePoints(FaceD,898,764,1825,1725);

		memcpy(face_pixels.data+face_pixels.step*F,FaceF,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*U,FaceU,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*R,FaceR,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*L,FaceL,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*D,FaceD,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*B,FaceB,sizeof(float)*18);

		image_publisher =  it.advertise("/chris/output_image", 1);

		template_image  = cv::imread("/u/chris/template.pgm",0);


	}

	/**
	 *  fill in the pre-allocated float array with 9*(x,y) pairs
	 */
	void calculateFacePoints(float *facepts, int tx, int ty, int bx, int by) {
		int pt=0;
		for (int y=ty; y<by+1; y+=(by-ty)/2.0) {
			for (int x=tx; x<bx+1; x+=(bx-tx)/2.0) {
				facepts[pt]=x;
				facepts[pt+1]=y;
				pt+=2;
			}
		}
	}

	/**
	 *  swap the ordering of the face pts array so as to account for face rotation
	 */
	void rotateFacePoints(float facepts[], int number_of_turns) {
		if (number_of_turns == 0)
			return;
		float temp[18];
		memcpy(temp,facepts,sizeof(temp));
		facepts[0]=temp[12]; 	facepts[1]=temp[13];
		facepts[2]=temp[6]; 	facepts[3]=temp[7];
		facepts[4]=temp[0]; 	facepts[5]=temp[1];
		facepts[6]=temp[14]; 	facepts[7]=temp[15];
		facepts[8]=temp[8]; 	facepts[9]=temp[9];
		facepts[10]=temp[2]; 	facepts[11]=temp[3];
		facepts[12]=temp[16]; 	facepts[13]=temp[17];
		facepts[14]=temp[10]; 	facepts[15]=temp[11];
		facepts[16]=temp[4]; 	facepts[17]=temp[5];

		rotateFacePoints(facepts,number_of_turns-1);
	}

	bool inspectFace(Face f) {
		inspect_face = f;
		polled_camera::GetPolledImage image_request;

	    // Request and image.
		image_request.request.response_namespace = "/prosilica_req";
		shot = false;
	    ros::ServiceClient client = node.serviceClient<polled_camera::GetPolledImage>("/prosilica/request_image");
		if (client.call(image_request))  {
			ROS_INFO("Requesting prosilica image. \n");
		}
		else	{
			ROS_ERROR("Failed to call prosilica request service.");
			return false;
		}

		// Wait for an image to arrive.
		while (!shot)
			ros::spinOnce();

		ROS_INFO("Processing rubik's cube image.");
		ROS_INFO("Image in frame: %s",image_header.frame_id.c_str());


		// Look at the pixel colour around the point of each cubie, and
		// store the mean of each cubie into the cubie_colours matrix
//		cvCvtColor(image,image,CV_BGR2HSV);
		static const int region_size = 45;
		cv::Mat mask( image.height()+2,image.width()+2,CV_8U);
		cv::Mat image_mat(image,false);
		mask = (float)0;
		for (int i=0; i<9;i++) {
			int x=face_pixels.ptr<float>((int)inspect_face)[i*2];
			int y=face_pixels.ptr<float>((int)inspect_face)[i*2+1];

			// Get the mean in the area around that face...
			// The size of the region to look at is given by the variable "region_size"
			mask = (float)0;
//			cv::circle(mask, cv::Point(x+1,y+1), region_size, CV_RGB(255,255,255), -1 );
			cv::floodFill(image_mat,mask,cv::Point(x,y),CV_RGB(255,255,255),0,CV_RGB(3,3,3),CV_RGB(3,3,3), cv::FLOODFILL_MASK_ONLY);
			cv::Scalar avg = cv::mean(image_mat,mask(cv::Rect(1,1,image_mat.cols,image_mat.rows)));

			// Draw it for debug...
			cv::circle(image_mat, cv::Point(x,y), region_size+1, CV_RGB(0,0,0), -1 );
			cv::circle(image_mat, cv::Point(x,y), region_size, avg, -1 );
			cubie_colours.ptr<float>((int)inspect_face*9 + i)[0] = avg.val[0];
			cubie_colours.ptr<float>((int)inspect_face*9 + i)[1] = avg.val[1];
			cubie_colours.ptr<float>((int)inspect_face*9 + i)[2] = avg.val[2];
		}


		// Save the face image for debuging...
		std::string debug_save_name;
		switch (inspect_face) {
			case F:
				debug_save_name = "image_F.jpg";
				break;
			case U:
				debug_save_name = "image_U.jpg";
				break;
			case L:
				debug_save_name = "image_L.jpg";
				break;
			case R:
				debug_save_name = "image_R.jpg";
				break;
			case D:
				debug_save_name = "image_D.jpg";
				break;
			case B:
				debug_save_name = "image_B.jpg";
				break;
		}

//		cvCircle(image,cvPoint(maxiP.x,maxiP.y),template_image.rows * (image.width() / image_resize.rows) /2.0,CV_RGB(255,0,0),5);
		cvSaveImage(debug_save_name.c_str(),image);
//		cv::imwrite("cubed.png",image_resize_grey);
		ROS_INFO("Saved image as %s",debug_save_name.c_str());

		// Publish the debug image
//		IplImage image_resize_ipl = image_resize_grey;
//		sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&image_resize_ipl, "mono8");
//		image_publisher.publish(msg);

		return true;

	}

	// Call after all the cubies have been filled in
	void calibrateColours() {
		ROS_INFO("Calibrating colours.");
		std::cout << "\n\n\n";
		for (int i=0;i<54;i++) {
			std::cout << cubie_colours.ptr<float>(i)[0] << "\t" << cubie_colours.ptr<float>(i)[1] << "\t" << cubie_colours.ptr<float>(i)[2] << "\n";
		}
		std::cout << "\n\n\n";

		int index=0;
		float avg[3];
		for (int j=0; j<6; j++) {
			avg[0]=avg[1]=avg[2]=0;
			for (int i=0; i< 9;i++) {
				avg[0]=avg[0]+cubie_colours.ptr<float>(index)[0];
				avg[1]=avg[1]+cubie_colours.ptr<float>(index)[1];
				avg[2]=avg[2]+cubie_colours.ptr<float>(index)[2];
				index++;
			}
			avg[0]=avg[0]/9.0;
			avg[1]=avg[1]/9.0;
			avg[2]=avg[2]/9.0;
			centres[j*3 + 0] = avg[0];
			centres[j*3 + 1] = avg[1];
			centres[j*3 + 2] = avg[2];
			ROS_INFO("Average: %f\t%f\t%f",avg[0],avg[1],avg[2]);
		}
	}

	// Constructs a cube description from the cubie colours
	std::string constructDescription() {
		ROS_INFO("Constructing description of rubiks cube...");
		// Run kmeans on all the cubies, split to 6 clusters
//		cubie_colours = cubie_colours.reshape(3,54); // reshape so each cubie is on a row of its own
		cv::Mat labels(54,1,CV_32S);
		cv::Mat centers(6, 1, CV_32FC3);//,centres);
//		centres[0] = (float*)centers.data;
//
		ROS_INFO("kmeans on %d samples.",cubie_colours.rows);
		cv::kmeans(cubie_colours,6,labels,cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100000, 0.10),
	               3, cv::KMEANS_RANDOM_CENTERS, &centers);
		ROS_INFO("Kmeans done.");

//		float centers[] = {};
		cv::Mat similarity(54,6,CV_32FC1);
		double distance=1e10;
		int index=0;
		for (int i=0; i<54; i++) {
			distance = 1e100;
			index = 0;
//			similarity.ptr<float>(i)[0] = i;
			for (int j=0; j<6; j++) {
				double d = sqrt(pow(centers.ptr<float>(j)[0]-cubie_colours.ptr<float>(i)[0], 2) +
								pow(centers.ptr<float>(j)[1]-cubie_colours.ptr<float>(i)[1], 2) +
								pow(centers.ptr<float>(j)[2]-cubie_colours.ptr<float>(i)[2], 2)
								);
//				std::cout <<"dist = "<<d<<std::endl;
				similarity.ptr<float>(i)[j] = d;
			}
		}
		cv::Mat similarity_sort;//(54,7,CV_32S);

		std::cout << "Trying to sort.\n";
		cv::sortIdx(similarity, similarity_sort, CV_SORT_ASCENDING + CV_SORT_EVERY_COLUMN );
		std::cout << "Sorted one\n";

		for (int i=0; i<6; i++) {
			for (int j=0;j<9;j++) {
				std::cout << "best index for colour " << i << ": " << similarity_sort.ptr<int>(j)[i] << std::endl;
				labels.at<int>(similarity_sort.ptr<int>(j)[i]) = i;
				for (int k=0;k<6;k++) {
					similarity.ptr<float>(similarity_sort.ptr<int>(j)[i])[k]=1e100;
				}
			}
			cv::sortIdx(similarity, similarity_sort, CV_SORT_ASCENDING + CV_SORT_EVERY_COLUMN );
		}

//
//		for (int i=0;i<54; i++) {
//			std::cout << "Label: " << (int)labels.at<int>(i) << std::endl;
//		}
//		labels = labels.reshape(0,6);


		 //A debug "colour chart"
		OcvImage <RGBPixel< unsigned char> > chart(7, 24, IPL_DEPTH_8U, 3);
		uint j=0;
		uint step=0;
		for (int i=0;i<6 ;i++) {
			for (int l=0;l<3;l++) {
				for (int k=0;k<3;k++) {
					// The actual looked at colour
					chart[i*3 + l + step][k].r = cubie_colours.ptr<float>(j)[2];
					chart[i*3 + l + step][k].g = cubie_colours.ptr<float>(j)[1];
					chart[i*3 + l + step][k].b = cubie_colours.ptr<float>(j)[0];

//					// The label colour applied
					chart[i*3 + l + step][k+4].r = centers.ptr<float>(labels.at<int>(j))[2];
					chart[i*3 + l + step][k+4].g = centers.ptr<float>(labels.at<int>(j))[1];
					chart[i*3 + l + step][k+4].b = centers.ptr<float>(labels.at<int>(j))[0];
					j++;
				}
			}
			step++; // to leave a line between faces...
		}
		cvSaveImage("debugchart.png",chart);
//		cv::imwrite("debg2.png",centers);

		std::string description;
		std::string codes[]={"r","g","b","y","o","w"};
		description = "";
		description.append("U:");
		for (int i=0;i<9;i++) {
			description.append(codes[labels.at<int>(U * 9 + i)]);
		}
		description.append(" ");
		description.append("F:");
		for (int i=0;i<9;i++) {
			description.append(codes[labels.at<int>(F * 9 + i)]);
		}
		description.append(" ");

		description.append("D:");
		for (int i=0;i<9;i++) {
			description.append(codes[labels.at<int>(D * 9 + i)]);
		}
		description.append(" ");

		description.append("B:");
		for (int i=0;i<9;i++) {
			description.append(codes[labels.at<int>(B * 9 + i)]);
		}
		description.append(" ");

		description.append("L:");
		for (int i=0;i<9;i++) {
			description.append(codes[labels.at<int>(L * 9 + i)]);
		}
		description.append(" ");
		description.append("R:");
		for (int i=0;i<9;i++) {
			description.append(codes[labels.at<int>(R * 9 + i)]);
		}
		description.append(" ");

		ROS_INFO("Cube State: %s",description.c_str());
		return description;
	}

	void processImage() {
		if (shot)
			return;
		shot=true;
	}
};





struct SolutionExec {
	std::vector<std::string> solution;
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
	//            const rubiks_graph::MoveToStateActionResultConstPtr& result)
			boost::shared_ptr<const rubiks_graph::MoveToStateResult_<std::allocator<void> > > result)
	{

	  ROS_INFO("Finished in state [%s]", state.toString().c_str());
	//  ROS_INFO("Answer: %s", result->result.info.c_str());
	  ROS_INFO("---------------------------");
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("---------------------------");
		ROS_INFO("Goal just went active");
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const rubiks_graph::MoveToStateFeedbackConstPtr& feedback)
	{
	  ROS_INFO("Got Feedback: position = %d\tstring=%s", feedback->action_number, feedback->status.c_str());
	}
};


int main(int argc, char** argv) {

	ros::init(argc, argv, "RubikInspector");
	ros::NodeHandle node;


	RubiksImageCallback rubikSolver(node);//,"/prosilica_req/image_color", "/prosilica_req/camera_info");

	actionlib::SimpleActionClient<rubiks_graph::MoveToStateAction> ac("/rubiks_graph", true);
	ac.waitForServer(); //will wait for infinite time

	rubiks_graph::MoveToStateGoal goal;
	bool finished_before_timeout;


	std::string input;

//	goal.actions.clear();
//	goal.actions.push_back(std::string("INIT_CLOSED"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult();

	std::cout << "Enter 'ok'  to open and init ...";
	std::cin >> input;

	if(input.compare(std::string("ok")) == 0 ) {
		// Need to reset in hand pose - open gripper
		goal.actions.clear();
		goal.actions.push_back(std::string("INIT"));
		ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
		finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
		std::cout << "Enter *  when set in hand ...";
		std::cin >> input;
	}


	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_F"));
	ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::F);

	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_U"));
	ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::U);

	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_B"));
	ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::B);

	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_L"));
	ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::L);

	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_R"));
	ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::R);

	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_D"));
	ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::D);




	std::string cubestate;
	cubestate = rubikSolver.constructDescription();

	std::cout << "Should solve(yes|no)?\n";
//	std::cin >> input;
	if (true) { //input.compare("yes")==0) {
	//	rubiks_solver::SolveService

		ros::ServiceClient client = node.serviceClient<rubiks_solver::SolveService>("rubiks_solve");
		rubiks_solver::SolveService srv;
		srv.request.cubeState=cubestate;
		srv.request.timeout=10;
		if (client.call(srv))
		{
	//		ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service to solve the cube.");
			return 1;
		}

		ROS_INFO("Status: %s",srv.response.status.c_str());
		ROS_INFO("Solution contains %d moves.",srv.response.solution.size());
		if (!(srv.response.success)) {
		} else {
			SolutionExec exec;
			exec.solution = srv.response.solution;
			goal.actions = srv.response.solution;
			ac.sendGoal(goal);//, 0,//boost::bind(&SolutionExec::doneCb, &exec, _1, _2),
//					boost::bind(&SolutionExec::activeCb, &exec, _1),
//					boost::bind(&SolutionExec::feedbackCb, &exec, _1));//, &doneCb, &activeCb, &feedbackCb);
			finished_before_timeout = ac.waitForResult();//ros::Duration(600.0)
			goal.actions.clear();
			goal.actions.push_back(std::string("DANCE"));
			ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
			finished_before_timeout = ac.waitForResult(ros::Duration());
		}


		while (node.ok()) {
			ros::spinOnce();
		}
		return 1;
	}
}

// Scrap land of waste...

//		// Project the cube points into the image.....
//		cv::Point3d grip;
//		cv::Point2d project;
//		transforms.lookupTransform(image_header.frame_id,"/l_gripper_tool_frame",image_header.stamp, transform);
////		std::cout << "" << transform.getOrigin().getX() << std::endl;
//		grip.x = transform.getOrigin().getX();
//		grip.y = transform.getOrigin().getY();
//		grip.z = transform.getOrigin().getZ();
//		pinhole.project3dToPixel(grip,project);
//		cvCircle(image,cvPoint(project.x, project.y),5,CV_RGB(255,0,255),2);
//		pinhole.unrectifyPoint(project,project);
//		cvCircle(image,cvPoint(project.x, project.y),5,CV_RGB(255,0,0), 4);
//
//		transforms.lookupTransform(image_header.frame_id,"/l_gripper_led_frame",image_header.stamp, transform);
////		transforms.transformPoint()
////		std::cout << "" << transform.getOrigin().getX() << std::endl;
//		grip.x = transform.getOrigin().getX();
//		grip.y = transform.getOrigin().getY();
//		grip.z = transform.getOrigin().getZ();
//		pinhole.project3dToPixel(grip,project);
//		cvCircle(image,cvPoint(project.x, project.y),5,CV_RGB(255,255,0),2);
//		pinhole.unrectifyPoint(project,project);
//		cvCircle(image,cvPoint(project.x, project.y),5,CV_RGB(0,255,255),4);


//	goal.actions.clear();
//	//'F', 'Bi', 'R', 'L', 'L2'
//	//'F2', 'B', 'Ui', 'R', 'D', 'L2', 'F2', 'Ri', 'U', 'Di'
	//['R', 'B2', 'U', 'R', 'Ui', 'L', 'F', 'Li', 'D', 'Ri', 'U', 'L2', 'U', 'R2', 'F2', 'D', 'F2', 'Di', 'L2', 'D2', 'F2', 'D']
//	goal.actions.push_back(std::string("R"));
//	goal.actions.push_back(std::string("B2"));
//	goal.actions.push_back(std::string("U"));
//	goal.actions.push_back(std::string("R"));
//	goal.actions.push_back(std::string("Ui"));
//	goal.actions.push_back(std::string("L"));
//	goal.actions.push_back(std::string("F"));
//	goal.actions.push_back(std::string("Li"));
//	goal.actions.push_back(std::string("D"));
//	goal.actions.push_back(std::string("Ri"));
//	goal.actions.push_back(std::string("U"));
//	goal.actions.push_back(std::string("L2"));
//	goal.actions.push_back(std::string("U"));
//	goal.actions.push_back(std::string("R2"));
//	goal.actions.push_back(std::string("F2"));
//	goal.actions.push_back(std::string("D"));
//	goal.actions.push_back(std::string("F2"));
//	goal.actions.push_back(std::string("Di"));
//	goal.actions.push_back(std::string("L2"));
//	goal.actions.push_back(std::string("D2"));
//	goal.actions.push_back(std::string("F2"));
//	goal.actions.push_back(std::string("D"));

//
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(6000.0));
//
//	rubikSolver.calibrateColours();
//
//
//	////////////////////////////////
//	// Calibration over....
//	///////////////////////////////
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INIT_CLOSE"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//
//	std::cout << "Press something and return" << std::endl;
//	std::cin >> input;
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INIT"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//
//	std::cout << "Press something and return" << std::endl;
//	std::cin >> input;
//
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INSPECT_F"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//	rubikSolver.inspectFace(RubiksImageCallback::F);
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INSPECT_U"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//	rubikSolver.inspectFace(RubiksImageCallback::U);
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INSPECT_B"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//	rubikSolver.inspectFace(RubiksImageCallback::B);
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INSPECT_L"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//	rubikSolver.inspectFace(RubiksImageCallback::L);
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INSPECT_R"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//	rubikSolver.inspectFace(RubiksImageCallback::R);
//
//	goal.actions.clear();
//	goal.actions.push_back(std::string("INSPECT_D"));
//	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
//	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
//	rubikSolver.inspectFace(RubiksImageCallback::D);

//		// Images..
//		cv::Mat image_resize(768,1024,CV_8UC3);
//		cv::Mat image_resize_grey(image_resize.rows, image_resize.cols,CV_8UC1);
//		cv::Mat image_resize_hsv(image_resize.rows, image_resize.cols,CV_8UC3);
//
//		// Find the size and location approximate info
//
//		int scale=face_pixels.ptr<float>((int)inspect_face)[2];
//		int x=face_pixels.ptr<float>((int)inspect_face)[0];
//		int y=face_pixels.ptr<float>((int)inspect_face)[1];
//		cvCircle(image,cvPoint(x,y),scale,CV_RGB(255,255,0),1);
//		scale = scale / (float) image.width() * image_resize.cols;
//		x=x / (float)image.width() * image_resize.cols - scale / 2.0;
//		y=y / (float)image.height() * image_resize.rows - scale / 2.0;
//		cv::resize(template_image,template_image,cv::Size(scale,scale),0,0);
//
//		// First find the face in the image
//
//		cv::resize(cv::Mat(image.iplimage,false),image_resize,cv::Size(image_resize.cols, image_resize.rows),0,0);
//		cv::cvtColor(image_resize,image_resize_grey,CV_BGR2GRAY);
//		cv::cvtColor(image_resize,image_resize_hsv,CV_BGR2HSV);
//		std::vector<cv::Mat> hsv_channels;
//		cv::split(image_resize_hsv,hsv_channels);
//		image_resize_grey = hsv_channels[1];
////		cv::Canny(image_resize_grey, image_resize_grey, 30, 40);
////		cv::Mat fx(image_resize.rows, image_resize.cols, CV_32F);
////		cv::Mat fy(image_resize.rows, image_resize.cols, CV_32F);
////		cv::Mat g(image_resize.rows, image_resize.cols, CV_32F);
////		cv::Sobel(image_resize_grey,fx,CV_32F,1,0,3);
////		cv::Sobel(image_resize_grey,fy,CV_32F,0,1,3);
////		cv::multiply(fx,fx,fx);
////		cv::multiply(fy,fy,fy);
////		cv::add(fx,fy,fx);
////		cv::sqrt(fx,g);
////		cv::convertScaleAbs(g,image_resize_grey);
//		cv::Mat template_result(image_resize.rows - template_image.rows + 1, image_resize.cols - template_image.cols + 1,CV_32FC1);
//		cv::matchTemplate(image_resize_grey,template_image,template_result,CV_TM_SQDIFF_NORMED);
//		double mini,maxi;
//		cv::Point miniP, maxiP;
//
//
//		cv::Mat mask(template_result.rows,template_result.cols,CV_8UC1);
//		mask=(float)0;
//
//		cv::circle(mask,cv::Point(x,y),2.0*scale,cv::Scalar(255,255,255),-1);
//		cv::minMaxLoc(template_result, &mini, &maxi, &maxiP, &miniP,mask);	// maxi and mini are swapped for testing...
//		cv::Mat res(template_image.rows,template_image.cols,CV_8UC1);
//		cv::convertScaleAbs(template_result,res,255);
//		image_resize_grey = res;
//		std::cout << "Best location " << maxiP.x << " " << maxiP.y << "; strength=" << maxi << std::endl;
//		// Change maxiP to be in the original larger image not smaller image
//		maxiP.x = (maxiP.x + (template_image.cols/2.0))/image_resize_grey.cols * image.width();
//		maxiP.y = (maxiP.y + (template_image.rows/2.0))/image_resize_grey.rows * image.height();
//		std::cout << "Best location HD " << maxiP.x << " " << maxiP.y << std::endl;
//
//		// From the best match location, place a grid to mark the cubelet locations
//		int template_size = template_image.cols * (image.width() / (float)image_resize.cols);
//		int square_step = template_size / 3.0;
//		for (int y=square_step/2.0; y< 3*square_step; y+=square_step) {
//			for (int x=square_step/2.0; x< 3*square_step; x+=square_step) {
//				cvCircle(image,cvPoint(maxiP.x + x, maxiP.y+y),20,CV_RGB(0,0,255),-1);
//			}
//		}
//
//		cv::circle(image_resize_grey,cv::Point(x,y),scale,cv::Scalar(0,0,0),5);

