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

#include <algorithm>


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

	std_msgs::Header image_header;

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
	image_transport::ImageTransport it;

//	float centres[6*3];
	cv::Mat centers;

	cv::Mat labels;


	RubiksImageCallback(ros::NodeHandle &masternode)  : ImageCallback(masternode, "/prosilica_req/image_color", "/prosilica_req/camera_info"),
//	RubiksImageCallback(ros::NodeHandle &masternode)  : ImageCallback(masternode, "/prosilica_req/image_rect_color", "/prosilica_req/camera_info"),
//	RubiksImageCallback(ros::NodeHandle &masternode)  : ImageCallback(masternode, "/wide_stereo/right/image_rect_color", "/wide_stereo/right/camera_info"),
			face_pixels(6, 18, CV_32FC1),
			cubie_colours(54,1, CV_32FC3),
			it(masternode),
			centers(6, 1, CV_32FC3),
			labels(54,1,CV_32S){

		shot = false;

		// Each face has a float array of the format
		// [ x_0, y_0, x_1, y_1, .... , x_8, y_8 ]
		// ie. an 18d float array with the centre positions of each cubie in the
		// image to be used as a seed points to be improved by the square detection..
		float FaceF[18], FaceR[18], FaceL[18], FaceU[18], FaceD[18], FaceB[18];

		// Fill in the image point estimates...
		// The utility calculateFacePoints will fill in the 18d array with the
		// points given only the top left corner point and the bottom right corner.
		//
		calculateFacePoints(FaceF,991,700,1255,909);
		calculateFacePoints(FaceB,1057,836,1281,1056);
		calculateFacePoints(FaceU,1154,729,1439,1028);
		calculateFacePoints(FaceR,875,987,1070,1195);
		rotateFacePoints(FaceR,2);	// The R, L and D faces are heald upside down by the robot, so need rotating..
		calculateFacePoints(FaceL,947,944,1138,1149);
		rotateFacePoints(FaceL,2);
		calculateFacePoints(FaceD,868,1415,1157,1733);
		rotateFacePoints(FaceD,3);

		// Copy the pixel points into the matrix - yes, could have put them there
		// straight away...
		memcpy(face_pixels.data+face_pixels.step*F,FaceF,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*U,FaceU,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*R,FaceR,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*L,FaceL,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*D,FaceD,sizeof(float)*18);
		memcpy(face_pixels.data+face_pixels.step*B,FaceB,sizeof(float)*18);



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


	// helper function:
	// finds a cosine of angle between vectors
	// from pt0->pt1 and from pt0->pt2
	double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
	{
	    double dx1 = pt1->x - pt0->x;
	    double dy1 = pt1->y - pt0->y;
	    double dx2 = pt2->x - pt0->x;
	    double dy2 = pt2->y - pt0->y;
	    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
	}
	// returns sequence of squares detected on the image.
	// the sequence is stored in the specified memory storage
	CvSeq* findSquares4(IplImage* img, CvMemStorage* storage) {
		const int thresh = 50;
		CvSeq* contours;
		int i, c, l, N = 11;
		CvSize sz = cvSize(img->width & -2, img->height & -2);
		IplImage* timg = cvCloneImage(img); // make a copy of input image
		IplImage* gray = cvCreateImage(sz, 8, 1);
		IplImage* pyr =
				cvCreateImage(cvSize(sz.width / 2, sz.height / 2), 8, 3);
		IplImage* tgray;
		CvSeq* result;
		double s, t;
		// create empty sequence that will contain points -
		// 4 points per square (the square's vertices)
		CvSeq* squares =
				cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), storage);

		// select the maximum ROI in the image
		// with the width and height divisible by 2
		cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));

		// down-scale and upscale the image to filter out the noise
		cvPyrDown(timg, pyr, 7);
		cvPyrUp(pyr, timg, 7);
		tgray = cvCreateImage(sz, 8, 1);

		// find squares in every color plane of the image
		for (c = 0; c < 3; c++) {
			// extract the c-th color plane
			cvSetImageCOI(timg, c + 1);
			cvCopy(timg, tgray, 0);

			// try several threshold levels
			for (l = 0; l < N; l++) {
				// hack: use Canny instead of zero threshold level.
				// Canny helps to catch squares with gradient shading
				if (l == 0) {
					// apply Canny. Take the upper threshold from slider
					// and set the lower to 0 (which forces edges merging)
					cvCanny(tgray, gray, 0, thresh, 5);
					// dilate canny output to remove potential
					// holes between edge segments
					cvDilate(gray, gray, 0, 1);
				} else {
					// apply threshold if l!=0:
					//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
					cvThreshold(tgray, gray, (l + 1) * 255 / N, 255,
							CV_THRESH_BINARY);
				}

				// find contours and store them all as a list
				cvFindContours(gray, storage, &contours, sizeof(CvContour),
						CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

				// test each contour
				while (contours) {
					// approximate contour with accuracy proportional
					// to the contour perimeter
					result = cvApproxPoly(contours, sizeof(CvContour), storage,
							CV_POLY_APPROX_DP, cvContourPerimeter(contours)
									* 0.02, 0);
					// square contours should have 4 vertices after approximation
					// relatively large area (to filter out noisy contours)
					// and be convex.
					// Note: absolute value of an area is used because
					// area may be positive or negative - in accordance with the
					// contour orientation
					if (result->total == 4 && fabs(cvContourArea(result,
							CV_WHOLE_SEQ)) > 1000 && cvCheckContourConvexity(
							result)) {
						s = 0;

						for (i = 0; i < 5; i++) {
							// find minimum angle between joint
							// edges (maximum of cosine)
							if (i >= 2) {
								t = fabs(angle((CvPoint*) cvGetSeqElem(result,
										i), (CvPoint*) cvGetSeqElem(result, i
										- 2), (CvPoint*) cvGetSeqElem(result, i
										- 1)));
								s = s > t ? s : t;
							}
						}

						// if cosines of all angles are small
						// (all angles are ~90 degree) then write quandrange
						// vertices to resultant sequence
						if (s < 0.3)
							for (i = 0; i < 4; i++)
								cvSeqPush(squares, (CvPoint*) cvGetSeqElem(
										result, i));
					}

					// take the next contour
					contours = contours->h_next;
				}
			}
		}

		// release all the temporary images
		cvReleaseImage(&gray);
		cvReleaseImage(&pyr);
		cvReleaseImage(&tgray);
		cvReleaseImage(&timg);

		return squares;
	}

	/**
	 * find the mean colour of the pixels in the suplied square (vertices)
	 */
	CvScalar computeSquareMean(IplImage *image, CvPoint *vertices) {
		IplImage *mask = cvCreateImage(cvSize(image->width, image->height),IPL_DEPTH_8U,1);
		cvSet(mask,CV_RGB(0,0,0));
		const int verts = 4;
		cvFillPoly( mask, &vertices, &verts, 1, CV_RGB(255,255,255)  );
		CvScalar avg = cvAvg(image,mask);
		cvReleaseImage(&mask);
		return avg;
	}

	/**
	 * find the median colour of the pixels in the suplied square (vertices)
	 * probably not suitable for compuational speed sensitive situations
	 */
	CvScalar computeSquareMedian(IplImage *image, CvPoint *vertices) {
		IplImage *mask = cvCreateImage(cvSize(image->width, image->height),IPL_DEPTH_8U,1);
		cvSet(mask,CV_RGB(0,0,0));
		const int verts = 4;
		cvFillPoly( mask, &vertices, &verts, 1, CV_RGB(255,255,255));

		int elements = abs(vertices[0].x-vertices[2].x) * abs(vertices[0].y*vertices[2].y);
		std::vector<uchar> r;
		std::vector<uchar> g;
		std::vector<uchar> b;
		r.reserve(elements);
		g.reserve(elements);
		b.reserve(elements);
		for (int i=0, colour_ptr=0; i < image->width * image->height; i++, colour_ptr+=3) {
			if (mask->imageData[i]) { // chicken pie
				r.insert(r.end(), image->imageData[colour_ptr]);
				g.insert(g.end(), image->imageData[colour_ptr+1]);
				b.insert(b.end(), image->imageData[colour_ptr+2]);
			}
		}
		std::sort(r.begin(),r.end());
		std::sort(g.begin(),g.end());
		std::sort(b.begin(),b.end());

		CvScalar avg;//

		avg.val[0]=r[r.size()/2];
		avg.val[1]=g[g.size()/2];
		avg.val[2]=b[b.size()/2];

		cvReleaseImage(&mask);
		return avg;
	}

	/**
	 * inspect the given face to find out what colours the cubies are
	 * assumes that the cube is already being heald in the exact place in the
	 * image..
	 */
	bool inspectFace(Face f) {
		inspect_face = f;
		polled_camera::GetPolledImage image_request;

	    // Request an image.
		image_request.request.response_namespace = "/prosilica_req";
		shot = false;
	    ros::ServiceClient client = node.serviceClient<polled_camera::GetPolledImage>("/prosilica/request_image");
		if (client.call(image_request))  {
//			ROS_INFO("Requesting prosilica image. \n");
		}
		else	{
			ROS_ERROR("Failed to call prosilica request service.");
			return false;
		}

		// Wait for an image to arrive.
		while (!shot)
			ros::spinOnce();

//		ROS_INFO("Processing rubik's cube image.");
//		ROS_INFO("Image in frame: %s",image_header.frame_id.c_str());


		// Look at the pixel colour around the point of each cubie, and
		// store the colour of each cubie into the cubie_colours matrix
		// Do this by finding the nearest square in the image to the
		// cubie seed point, and taking the median value of those pixels

		// (1) create memory storage that will contain all the dynamic data
		// for the square detection routine
		CvMemStorage *storage = cvCreateMemStorage(0);
		// squares size... along diagonal...
		int square_size;// = abs(face_pixels.ptr<float>((int)inspect_face)[2] - face_pixels.ptr<float>((int)inspect_face)[0]);
		square_size = sqrt( ((face_pixels.ptr<float>((int)inspect_face)[8] - face_pixels.ptr<float>((int)inspect_face)[0]) * (face_pixels.ptr<float>((int)inspect_face)[8] - face_pixels.ptr<float>((int)inspect_face)[0])) +
				((face_pixels.ptr<float>((int)inspect_face)[9] - face_pixels.ptr<float>((int)inspect_face)[1]) * (face_pixels.ptr<float>((int)inspect_face)[9] - face_pixels.ptr<float>((int)inspect_face)[1])) );

		// (2) detect all the squares in the image
//		ROS_INFO(" InspectFace: Detecting squares in image.....");
		CvSeq *squares = findSquares4( image, storage );

		// (3) for each cubie, find the closest square...
		for (int i=0; i<9;i++) {
			int x=face_pixels.ptr<float>((int)inspect_face)[i*2];
			int y=face_pixels.ptr<float>((int)inspect_face)[i*2+1];

			// Look for the closest and smallest square to the point
			float bestDistance = 1e100;
			CvPoint bestSquare[4];

			CvSeqReader reader;
			// initialize reader of the sequence
			cvStartReadSeq( squares, &reader, 0 );
			// read 4 sequence elements at a time (all vertices of a square)
			for( int j = 0; j < squares->total; j += 4 ) {
				CvPoint pt[4];  //, *rect = pt;

				// read 4 vertices
				CV_READ_SEQ_ELEM( pt[0], reader );
				CV_READ_SEQ_ELEM( pt[1], reader );
				CV_READ_SEQ_ELEM( pt[2], reader );
				CV_READ_SEQ_ELEM( pt[3], reader );

				// find mean vertices - centre of square
				CvPoint centre;
				centre.x = (pt[0].x + pt[1].x + pt[2].x + pt[3].x) / 4.0;
				centre.y = (pt[0].y + pt[1].y + pt[2].y + pt[3].y) / 4.0;

				int size = 2 * sqrt( (centre.x - pt[1].x)*(centre.x - pt[1].x)  + (centre.y - pt[1].y)*(centre.y - pt[1].y) );

				// if the square is bigger than 2 * should be size then dismiss it
				if (size > 2.0*square_size)
					continue;
				// if the square is smaller than 0.2 * should be size then dismiss it
				if (size < 0.2*square_size)
					continue;
//std::cout << "square for considering..\n";
				// Find the distance^2 from the centre to the original point
				float dist = sqrt((centre.x-x)*(centre.x-x) + (centre.y-y)*(centre.y-y));
//				image.writeFormatted(centre.x,centre.y,"%d",CV_RGB(255,255,0),1,1,j);
				image.writeFormatted(x,y,"%d",CV_RGB(0,0,0),1,1,i);

				if (dist < bestDistance) {
					bestDistance = dist;
//					bestSquare = j;
					for (int k=0;k<4;k++)
						bestSquare[k] = pt[k];
				}
			}
			// draw the square as a closed polyline
			const int verts = 4;
//			ROS_INFO("Bests square: %d",bestSquare);
			CvPoint *rect = bestSquare;//(CvPoint*)(squares->first->data+(bestSquare*sizeof(CvPoint)));
//			std::cout << "Best dist = " << bestDistance << std::endl;

			// (4) find the median of the pixel values in the square
//			CvScalar avg = computeSquareMean(image,rect);
			CvScalar avg = computeSquareMedian(image,rect);

			// (5) Store the colour in the cubie_colours member
			cubie_colours.ptr<float>((int)inspect_face*9 + i)[0] = avg.val[0];
			cubie_colours.ptr<float>((int)inspect_face*9 + i)[1] = avg.val[1];
			cubie_colours.ptr<float>((int)inspect_face*9 + i)[2] = avg.val[2];

			// Visualise
			CvPoint centre;
			centre.x = (bestSquare[0].x + bestSquare[1].x + bestSquare[2].x + bestSquare[3].x) / 4.0;
			centre.y = (bestSquare[0].y + bestSquare[1].y + bestSquare[2].y + bestSquare[3].y) / 4.0;
			image.writeFormatted(centre.x,centre.y,"%d",CV_RGB(255,255,255),2,4,i);
			image.writeFormatted(centre.x,centre.y,"%d",CV_RGB(255,255,255),2,2,i);
			cvPolyLine( image, &rect, &verts, 1, 1, CV_RGB(0,255,0), 3, CV_AA, 0 );

		}
		// Clear the square detector storage
		cvClearMemStorage( storage );

		// Save the face image for debuging...
//		std::string debug_save_name;
//		switch (inspect_face) {
//			case F:
//				debug_save_name = "image_F.jpg";
//				break;
//			case U:
//				debug_save_name = "image_U.jpg";
//				break;
//			case L:
//				debug_save_name = "image_L.jpg";
//				break;
//			case R:
//				debug_save_name = "image_R.jpg";
//				break;
//			case D:
//				debug_save_name = "image_D.jpg";
//				break;
//			case B:
//				debug_save_name = "image_B.jpg";
//				break;
//		}
//		cvSaveImage(debug_save_name.c_str(),image);
//		ROS_INFO("Saved image as %s",debug_save_name.c_str());

		return true; // silly really, always returning true...

	}


	/**
	 * Constructs a cube description from the already filled in cubie colours
	 * member. To be called after all of the faces have been inspected.
	 * Returns a string of the format expected by the solver node (see the
	 * comments in the solver node source...)
	 */
	std::string constructDescription() {
//		ROS_INFO("Constructing description of Rubik's cube...");
		// Run kmeans on all the cubies, split to 6 clusters
		// then assign nine cubies to each of the clusters


//		ROS_INFO("kmeans on %d samples.",cubie_colours.rows); // Had better be 54 samples...
		cv::kmeans(cubie_colours,6,labels,cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100000, 0.10),
	               3, cv::KMEANS_RANDOM_CENTERS, &centers);
//		ROS_INFO("Kmeans done.");

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

//		std::cout << "Trying to sort.\n";
		cv::sortIdx(similarity, similarity_sort, CV_SORT_ASCENDING + CV_SORT_EVERY_COLUMN );
//		std::cout << "Sorted one\n";

		for (int i=0; i<6; i++) {
			for (int j=0;j<9;j++) {
//				std::cout << "best index for colour " << i << ": " << similarity_sort.ptr<int>(j)[i] << std::endl;
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


		 //A debug "colour chart" useful for seing what the results was
//		OcvImage <RGBPixel< unsigned char> > chart(7, 24, IPL_DEPTH_8U, 3);
//		uint j=0;
//		uint step=0;
//		for (int i=0;i<6 ;i++) {
//			for (int l=0;l<3;l++) {
//				for (int k=0;k<3;k++) {
//					// The actual looked at colour
//					chart[i*3 + l + step][k].r = cubie_colours.ptr<float>(j)[2];
//					chart[i*3 + l + step][k].g = cubie_colours.ptr<float>(j)[1];
//					chart[i*3 + l + step][k].b = cubie_colours.ptr<float>(j)[0];
//
////					// The label colour applied
//					chart[i*3 + l + step][k+4].r = centers.ptr<float>(labels.at<int>(j))[2];
//					chart[i*3 + l + step][k+4].g = centers.ptr<float>(labels.at<int>(j))[1];
//					chart[i*3 + l + step][k+4].b = centers.ptr<float>(labels.at<int>(j))[0];
//					j++;
//				}
//			}
//			step++; // to leave a line between faces...
//		}
//		cvSaveImage("debugchart.png",chart);

		return makeDescriptionString();
	}

	std::string makeDescriptionString() {
		// Now create a string for the description
		// Since it is not important what the labels are, so long as all red are
		// labeled constantly etc, the description may call red blue and green yellow
		// for example...
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

	//
	void processImage() {
		if (shot)
			return;
		shot=true;
	}
};


void colour_selector_on_mouse(int event, int x, int y, int flags, void* param);
class ColourPicker {
public:
	OcvImage<RGBPixel<uchar> > image;
	int active_colour;
	cv::Mat colours;
	ColourPicker(cv::Mat &colours)  : image(100, 275, IPL_DEPTH_8U, 3) {
		this->colours = colours;
		cvNamedWindow("Colour Picker");
		cvSetMouseCallback("Colour Picker", &colour_selector_on_mouse, this);
		cvMoveWindow("Colour Picker",0,0);

		drawPalette();

		cvShowImage("Colour Picker", image);
		char key = cvWaitKey(0);
		if (key == 'r') {
			// reset the whole colour palette as it was screwed
			// BGR and RGB ness may be confused.
			std::vector<int> ids;
			for (int i=0;i<6;i++)
				ids.push_back(i);

			float dist;
			float best_dist=1e100;
			int best_i=0;
			// Find the closest to red
			for (int i=0;i<ids.size(); i++) {
				dist = ((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255)) +
						((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0)) +
						((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0));
				if (dist<best_dist) {
					best_i = i;
					best_dist = dist;
				}
			}
			colours.ptr<float>(ids[best_i])[2] = 255;
			colours.ptr<float>(ids[best_i])[1] = 0;
			colours.ptr<float>(ids[best_i])[0] = 0;
			ids.erase(ids.begin() + best_i);
			best_dist = 1e100;

			// Find the closest to green
			for (int i=0;i<ids.size(); i++) {
				dist = ((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0)) +
						((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255)) +
						((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0));
				if (dist<best_dist) {
					best_i = i;
					best_dist = dist;
				}
			}
			colours.ptr<float>(ids[best_i])[2] = 0;
			colours.ptr<float>(ids[best_i])[1] = 255;
			colours.ptr<float>(ids[best_i])[0] = 0;
			ids.erase(ids.begin() + best_i);
			best_dist = 1e100;

			// Find the closest to blue
			for (int i=0;i<ids.size(); i++) {
				dist = ((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0)) +
						((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0)) +
						((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255));
				if (dist<best_dist) {
					best_i = i;
					best_dist = dist;
				}
			}
			colours.ptr<float>(ids[best_i])[2] = 0;
			colours.ptr<float>(ids[best_i])[1] = 0;
			colours.ptr<float>(ids[best_i])[0] = 255;
			ids.erase(ids.begin() + best_i);
			best_dist = 1e100;

			// Find the closest to yellow
			for (int i=0;i<ids.size(); i++) {
				dist = ((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255)) +
						((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255)) +
						((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0));
				if (dist<best_dist) {
					best_i = i;
					best_dist = dist;
				}
			}
			colours.ptr<float>(ids[best_i])[2] = 255;
			colours.ptr<float>(ids[best_i])[1] = 255;
			colours.ptr<float>(ids[best_i])[0] = 0;
			ids.erase(ids.begin() + best_i);
			best_dist = 1e100;

			// Find the closest to orange
			for (int i=0;i<ids.size(); i++) {
				dist = ((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255)) +
						((colours.ptr<float>(ids[i])[2] - 126)*(colours.ptr<float>(ids[i])[2] - 126)) +
						((colours.ptr<float>(ids[i])[2] - 0)*(colours.ptr<float>(ids[i])[2] - 0));
				if (dist<best_dist) {
					best_i = i;
					best_dist = dist;
				}
			}
			colours.ptr<float>(ids[best_i])[2] = 255;
			colours.ptr<float>(ids[best_i])[1] = 126;
			colours.ptr<float>(ids[best_i])[0] = 0;
			ids.erase(ids.begin() + best_i);
			best_dist = 1e100;

			// Find the closest to white
			//should be 1 left

			for (int i=0;i<ids.size(); i++) {
				dist = ((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255)) +
						((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255)) +
						((colours.ptr<float>(ids[i])[2] - 255)*(colours.ptr<float>(ids[i])[2] - 255));
				if (dist<best_dist) {
					best_i = i;
					best_dist = dist;
				}
			}
			colours.ptr<float>(ids[best_i])[2] = 255;
			colours.ptr<float>(ids[best_i])[1] = 255;
			colours.ptr<float>(ids[best_i])[0] = 255;
			ids.erase(ids.begin() + best_i);
			best_dist = 1e100;


//			colours.ptr<float>(0)[2] = 255;
//			colours.ptr<float>(0)[1] = 0;
//			colours.ptr<float>(0)[0] = 0;
//
//			colours.ptr<float>(1)[2] = 0;
//			colours.ptr<float>(1)[1] = 255;
//			colours.ptr<float>(1)[0] = 0;
//
//			colours.ptr<float>(2)[2] = 0;
//			colours.ptr<float>(2)[1] = 0;
//			colours.ptr<float>(2)[0] = 255;
//
//			colours.ptr<float>(3)[2] = 255;
//			colours.ptr<float>(3)[1] = 126;
//			colours.ptr<float>(3)[0] = 0;
//
//			colours.ptr<float>(4)[2] = 255;
//			colours.ptr<float>(4)[1] = 255;
//			colours.ptr<float>(4)[0] = 0;
//
//			colours.ptr<float>(5)[2] = 255;
//			colours.ptr<float>(5)[1] = 255;
//			colours.ptr<float>(5)[0] = 255;
			drawPalette();
			cvShowImage("Colour Picker", image);

		}
	}

	void drawPalette() {
		// Draw the colour selector image
		image.writeString(2,20,"Colour",CV_RGB(255,255,255),0.9,2);
		image.writeString(2,45,"Picker",CV_RGB(255,255,255),0.9,2);
		int colour_id=0;
		for (int y=0;y<150;y++) {
			int yy = (y-(y % 50))/50;
			for (int x=0; x<100;x++) {
				int xx =(x-(x % 50))/50;
				colour_id = yy*2 + xx;
//				std::cout << "yy: " << yy << " xx: " << xx << "  colour id: " << colour_id << std::endl;
				image[y+50][x].b = colours.ptr<float>(colour_id)[0];
				image[y+50][x].g = colours.ptr<float>(colour_id)[1];
				image[y+50][x].r = colours.ptr<float>(colour_id)[2];
			}

		}
	}

	~ColourPicker() {
		cvDestroyWindow("Colour Picker");
		cvDestroyWindow("Colour Picker"); // die die die!
		cvDestroyWindow("Colour Picker"); // please?
		cvWaitKey(50);
	}

	void on_mouse(int event, int x, int y, int flags) {
		if (y<50)
			return;
		else
			y-=50;
		if (y>150)
			return;
		switch( event )		{
			case CV_EVENT_LBUTTONDOWN:
				int xx = (x-(x % 50))/50;
				int yy = (y-(y % 50))/50;
				active_colour = yy*2 + xx;

				for (int x=25; x<75; x++) {
					for (int y=220; y<270; y++)  {
						image[y][x].b = colours.ptr<float>(active_colour)[0];
						image[y][x].g = colours.ptr<float>(active_colour)[1];
						image[y][x].r = colours.ptr<float>(active_colour)[2];
					}
				}
//				image.writeFormatted(35,275,"Active Colour",CV_RGB(255,255,255),0.2,1);
				cvShowImage("Colour Picker", image);
				break;
		}
	}
};
void colour_selector_on_mouse(int event, int x, int y, int flags, void* param) {
	((ColourPicker*)param)->on_mouse(event,x,y,flags);
}

void painter_on_mouse(int event, int x, int y, int flags, void* param);
class FaceFixer {
public:
	OcvImage<RGBPixel<uchar> > painting_image;
	RubiksImageCallback::Face face;
	RubiksImageCallback *rubikSolver;
	ColourPicker *picker;

	FaceFixer(IplImage *face_image, ColourPicker *picker, RubiksImageCallback *rubikSolver, RubiksImageCallback::Face f)
	  : painting_image(150,150, IPL_DEPTH_8U, 3) {
		face = f;
		this->rubikSolver = rubikSolver;
		this->picker = picker;

		cvNamedWindow("Face Image");
		cvNamedWindow("Face Painting");
		cvSetMouseCallback("Face Painting", &painter_on_mouse, this);
		cvMoveWindow("Face Painting",120,0);
		cvMoveWindow("Face Image",400,0);

		updatePainting();

		cvShowImage("Face Painting", painting_image);
		cvShowImage("Face Image",face_image);
//		cvWaitKey(0);
		loopUntilHappy();
	}

	void updatePainting() {
		// Draw the face
		int cubie=0;
		for (int y=0;y<150;y++) {
			int yy = (y-(y % 50))/50;

			for (int x=0; x<150;x++) {
				int xx =(x-(x % 50))/50;
				cubie = yy*3 + xx;
//				std::cout << "yy: " << yy << " xx: " << xx << "  colour id: " << colour_id << std::endl;
				int index = rubikSolver->labels.at<int>(face * 9 + cubie);
				painting_image[y][x].b = picker->colours.ptr<float>(index)[0];
				painting_image[y][x].g = picker->colours.ptr<float>(index)[1];
				painting_image[y][x].r = picker->colours.ptr<float>(index)[2];
			}

		}
		// draww the lines
		CvPoint p1,p2;
		p2.x=0; p2.x=150;
		for (int i=0; i<4; i++) {
			p1.y=p2.y=i*50;
			cvLine(painting_image,p1,p2,CV_RGB(0,0,0),4);
		}
		p1.y=0;p2.y=150;
		for (int i=0; i<4; i++) {
			p1.x=p2.x=i*50;
			cvLine(painting_image,p1,p2,CV_RGB(0,0,0),4);
		}

	}

	// loop until the user hits 'a' to accept
	void loopUntilHappy() {
		bool running = true;
		while (running) {
			updatePainting();
			cvShowImage("Face Painting", painting_image);
			char key = cvWaitKey(10);
			if (key == 'a')
				running = false;
		}
	}


	void on_mouse(int event, int x, int y, int flags) {
		if (y>150)
			return;
		switch( event )		{
			case CV_EVENT_LBUTTONDOWN:
				int xx = (x-(x % 50))/50;
				int yy = (y-(y % 50))/50;
				int cubie = yy*3 + xx;
				rubikSolver->labels.at<int>(face * 9 + cubie) = picker->active_colour;
//				active_colour = yy*2 + xx;
//				std::cout << "Selected " << active_colour << "\n";
				break;
		}
	}

	~FaceFixer() {
		cvDestroyWindow("Face Painting");
		cvDestroyWindow("Face Image");
		cvWaitKey(10);
	}
};
void painter_on_mouse(int event, int x, int y, int flags, void* param) {
	((FaceFixer*)param)->on_mouse(event,x,y,flags);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "RubikInspector");
	ros::NodeHandle node;


	RubiksImageCallback rubikSolver(node);

	actionlib::SimpleActionClient<rubiks_graph::MoveToStateAction> ac("/rubiks_graph", true);
    ROS_INFO("Waiting for the rubiks_graph action server");
	ac.waitForServer(); //will hopefully not wait for infinite time
    ROS_INFO("The graph is ready");

	rubiks_graph::MoveToStateGoal goal;
	bool finished_before_timeout;

	OcvImage<BGRPixel<uchar> > imageF(640,480,IPL_DEPTH_8U,3);
	OcvImage<BGRPixel<uchar> > imageB(640,480,IPL_DEPTH_8U,3);
	OcvImage<BGRPixel<uchar> > imageU(640,480,IPL_DEPTH_8U,3);
	OcvImage<BGRPixel<uchar> > imageR(640,480,IPL_DEPTH_8U,3);
	OcvImage<BGRPixel<uchar> > imageL(640,480,IPL_DEPTH_8U,3);
	OcvImage<BGRPixel<uchar> > imageD(640,480,IPL_DEPTH_8U,3);


	std::string input;

	std::cout << "\nPR2 Rubik's Cube Solver\n-----------------------\n\n";

	std::cout << "?? Enter 'ok'  to open gripper and move to init pose:\n  \%: ";
	std::cin >> input;

	if(input.compare(std::string("ok")) == 0 ) {
		std::cout << "Opening gripper\n";
		// Need to reset in hand pose - open gripper
		goal.actions.clear();
		goal.actions.push_back(std::string("INIT"));
		ac.sendGoal(goal);
		finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
		std::cout << "?? Enter 'ok'  when set in hand:\n";
		do {
			std::cout << "  \%:  ";
			std::cin >> input;
		} while (input.compare(std::string("ok")) != 0 );
	}

	std::cout << "Inspecting cube face 'F'\n";
	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_F"));
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::F);
	cvResize(rubikSolver.image,imageF);

	std::cout << "Inspecting cube face 'U'\n";
	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_U"));
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::U);
	cvResize(rubikSolver.image,imageU);


	std::cout << "Inspecting cube face 'B'\n";
	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_B"));
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::B);
	cvResize(rubikSolver.image,imageB);


	std::cout << "Swapping hands and inspecting cube face 'L'\n";
	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_L"));
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::L);
	rubikSolver.image.rotate(rubikSolver.image.width()/2, rubikSolver.image.height()/2, M_PI);
	cvResize(rubikSolver.image,imageL);



	std::cout << "Inspecting cube face 'R'\n";
	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_R"));
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::R);
	rubikSolver.image.rotate(rubikSolver.image.width()/2, rubikSolver.image.height()/2, M_PI);
	cvResize(rubikSolver.image,imageR);


	std::cout << "Inspecting cube face 'D'\n";
	goal.actions.clear();
	goal.actions.push_back(std::string("INSPECT_D"));
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	rubikSolver.inspectFace(RubiksImageCallback::D);
	rubikSolver.image.rotate(rubikSolver.image.width()/2, rubikSolver.image.height()/2, M_PI_2);
	cvResize(rubikSolver.image,imageD);


	std::cout << "Constructing cube description.\n";
	std::string cubestate;
	cubestate = rubikSolver.constructDescription();

	std::cout << "Attempting to solve the cube with the auto-found colour description.\n";
	bool attemptSolve = true;
	ros::ServiceClient client = node.serviceClient<rubiks_solver::SolveService>("rubiks_solve");
	rubiks_solver::SolveService srv;
	while (attemptSolve) {
		srv.request.cubeState=cubestate;
		srv.request.timeout=10;
		if (client.call(srv))
		{
	//		ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			std::cerr << "Failed to call service to solve the cube. \nMaybe the solver node is not alive?";
			return 1;
		}


		if (!(srv.response.success)) {
			std::cout << "Solving failed, probably the colours of the cube were \nwrongly identified when inspecting.\n";
			std::cout << "Solver status returned: " << srv.response.status.c_str() << "\n";
			std::cout << "\n?? Do you want to try and manually rectify the colours? (yes | no)\n  \%:  ";
			std::cin >> input;
			if(input.compare(std::string("yes")) == 0 ) {
                std::cout<<"Press \"r\" to reset the palette (warning, the colors will be misplaced), \"a\" to accept a combination\n";
				// Display all of the images and allow clicking...
				// update 'cubestate'

				ColourPicker picker(rubikSolver.centers);
				FaceFixer f(imageF,&picker,&rubikSolver,RubiksImageCallback::F);
				FaceFixer u(imageU,&picker,&rubikSolver,RubiksImageCallback::U);
				FaceFixer b(imageB,&picker,&rubikSolver,RubiksImageCallback::B);
				FaceFixer l(imageL,&picker,&rubikSolver,RubiksImageCallback::L);
				FaceFixer r(imageR,&picker,&rubikSolver,RubiksImageCallback::R);
				FaceFixer d(imageD,&picker,&rubikSolver,RubiksImageCallback::D);

				cubestate = rubikSolver.makeDescriptionString();
				continue;
			}
		} else {
			std::cout << "Dead on, solving succeeded, now executing the solution.\n";
			goal.actions = srv.response.solution;
			ac.sendGoal(goal);
			finished_before_timeout = ac.waitForResult();
			goal.actions.clear();
			std::cout << "Cube solved. Doing celebratory dance.\n";
			goal.actions.push_back(std::string("DANCE"));
			ac.sendGoal(goal);
			finished_before_timeout = ac.waitForResult(ros::Duration());
			std::cout << "All done. :-) \n";
			return 1;
		}

		attemptSolve = false;
	}
	ros::spinOnce();

	return 1;
}

