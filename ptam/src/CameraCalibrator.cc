// Copyright 2008 Isis Innovation Limited
#include "ptam/OpenGL.h"
#include <gvars3/instances.h>
#include "ptam/CameraCalibrator.h"
#include <ptam/Params.h>
#include <TooN/SVD.h>
#include <fstream>
#include <stdlib.h>

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/utility.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

using namespace CVD;
using namespace std;
using namespace GVars3;

//Weiss{
Vector<NUMTRACKERCAMPARAMETERS> camparams;
//}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "cameracalibrator");
  ROS_INFO("starting CameraCalibrator with node name %s", ros::this_node::getName().c_str());

  cout << "  Welcome to CameraCalibrator " << endl;
  cout << "  -------------------------------------- " << endl;
  cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;

  GUI.StartParserThread();
  atexit(GUI.StopParserThread); // Clean up readline when program quits
  GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, SILENT);

  try
  {
    PtamParameters mPtamParameters;
    CameraCalibrator c;
    c.Run();
  }
  catch (CVD::Exceptions::All e)
  {
    cout << endl;
    cout << "!! Failed to run CameraCalibrator; got exception. " << endl;
    cout << "   Exception was: " << endl;
    cout << e.what << endl;
  }
}

void CameraCalibrator::imageCallback(const sensor_msgs::ImageConstPtr & img)
{

//  ROS_ASSERT(img->step == img->width);
  
  cv::Mat imgBW;
  if(img->encoding == sensor_msgs::image_encodings::RGB8)
  {
      // wrap around color data, and convert to bw image.
      cv::Mat imgColTmp = cv::Mat(img->height, img->width,CV_8UC3, (uchar*)img->data.data());
      cv::cvtColor(imgColTmp,imgBW,CV_RGB2GRAY);
  }
  else if(img->encoding == sensor_msgs::image_encodings::MONO8)
  {
      // just wrap around data... does not copy any image data.
      imgBW = cv::Mat(img->height, img->width,CV_8U, (uchar*)img->data.data());
  }
  else
  {
    // throw error
    ROS_ASSERT(img->encoding == sensor_msgs::image_encodings::RGB8 || img->encoding == sensor_msgs::image_encodings::MONO8);
  }
  
  // resize internal image if required
	if(mCurrentImage.size().x != img->width || mCurrentImage.size().y != img->height)
		mCurrentImage.resize(CVD::ImageRef(img->width, img->height));

  // copy data
	memcpy(mCurrentImage.data(),imgBW.data,img->width * img->height);

	mNewImage = true;
}

CameraCalibrator::CameraCalibrator() :
  mCamera("Camera"), mbDone(false), mCurrentImage(CVD::ImageRef(752, 480)), mDoOptimize(false), mNewImage(false)
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  mImageSub = it.subscribe("image", 1, &CameraCalibrator::imageCallback, this);
}

CameraCalibrator::~CameraCalibrator()
{
  delete mGLWindow;
}

void CameraCalibrator::init()
{
  mGLWindow = new GLWindow2(mCurrentImage.size(), "Camera Calibrator");

  mbDone = false;

  GUI.RegisterCommand("CameraCalibrator.GrabImages", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.GrabNextFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.ShowNext", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.SaveCalib", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("exit", GUICommandCallBack, this);

  GV3::Register(mgvnOptimizing, "CameraCalibrator.Optimize", 0, SILENT);
  GV3::Register(mgvnShowImage, "CameraCalibrator.Show", 0, SILENT);
  GV3::Register(mgvnDisableDistortion, "CameraCalibrator.NoDistortion", 0, SILENT);

  GUI.ParseLine("GLWindow.AddMenu CalibMenu");
  GUI.ParseLine("CalibMenu.AddMenuButton Live GrabImages CameraCalibrator.GrabImages");
  GUI.ParseLine("CalibMenu.AddMenuButton Live GrabFrame CameraCalibrator.GrabNextFrame");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Optimize \"CameraCalibrator.Optimize=1\"");
  GUI.ParseLine("CalibMenu.AddMenuToggle Live NoDist CameraCalibrator.NoDistortion");
  GUI.ParseLine("CalibMenu.AddMenuSlider Opti \"Show Img\" CameraCalibrator.Show 0 10");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Show Next\" CameraCalibrator.ShowNext");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Grab More\" CameraCalibrator.Optimize=0 ");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuToggle Opti NoDist CameraCalibrator.NoDistortion");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Save CameraCalibrator.SaveCalib");
  Reset();
}

void CameraCalibrator::Run()
{
  bool initialized = false;
  ros::Rate r(100);
  while (!mbDone && ros::ok())
  {

    ros::spinOnce();
    r.sleep();

    if (!initialized)
    {
      if (mNewImage)
      {
        init();
        initialized = true;
      }
      continue;
    }

    // Set up openGL
    mGLWindow->SetupViewport();
    mGLWindow->SetupVideoOrtho();
    mGLWindow->SetupVideoRasterPosAndZoom();

    if (mvCalibImgs.size() < 1)
      *mgvnOptimizing = false;

    if (!*mgvnOptimizing)
    {
      GUI.ParseLine("CalibMenu.ShowMenu Live");
      glDrawPixels(mCurrentImage);
      mDoOptimize = true; // set this so that optimization begins when "optimize" is pressed)

      if (mNewImage)
      {
        mNewImage = false;
        CalibImage c;
        if (c.MakeFromImage(mCurrentImage))
        {
          if (mbGrabNextFrame)
          {
            mvCalibImgs.push_back(c);
            mvCalibImgs.back().GuessInitialPose(mCamera);
            mvCalibImgs.back().Draw3DGrid(mCamera, false);
            mbGrabNextFrame = false;
          };
        }
      }
    }
    else
    {
      if(mDoOptimize)
        OptimizeOneStep();
      GUI.ParseLine("CalibMenu.ShowMenu Opti");
      int nToShow = *mgvnShowImage - 1;
      if (nToShow < 0)
        nToShow = 0;
      if (nToShow >= (int)mvCalibImgs.size())
        nToShow = mvCalibImgs.size() - 1;
      *mgvnShowImage = nToShow + 1;

      glDrawPixels(mvCalibImgs[nToShow].mim);
      mvCalibImgs[nToShow].Draw3DGrid(mCamera, true);
    }

    ostringstream ost;
    ost << "Camera Calibration: Grabbed " << mvCalibImgs.size() << " images." << endl;
    if (!*mgvnOptimizing)
    {
      ost << "Take snapshots of the calib grid with the \"GrabFrame\" button," << endl;
      ost << "and then press \"Optimize\"." << endl;
      ost << "Take enough shots (4+) at different angles to get points " << endl;
      ost << "into all parts of the image (corners too.) The whole grid " << endl;
      ost << "doesn't need to be visible so feel free to zoom in." << endl;
    }
    else
    {
      ost << "Current RMS pixel error is " << mdMeanPixelError << endl;
      ost << "Current camera params are  " << mCamera.mgvvCameraParams << endl;
      ost << "(That would be a pixel aspect ratio of " << mCamera.PixelAspectRatio() << ")" << endl;
      ost << "Check fit by looking through the grabbed images." << endl;
      ost << "RMS should go below 0.5, typically below 0.3 for a wide lens." << endl;
      ost << "Press \"save\" to save calibration to camera.cfg file and exit." << endl;
    }

    mGLWindow->DrawCaption(ost.str());
    mGLWindow->DrawMenus();
    mGLWindow->HandlePendingEvents();
    mGLWindow->swap_buffers();
  }
}

void CameraCalibrator::Reset()
{
  mCamera.mgvvCameraParams = ATANCamera::mvDefaultParams;
  if (*mgvnDisableDistortion)
    mCamera.DisableRadialDistortion();

  mCamera.SetImageSize(mCurrentImage.size());
  mbGrabNextFrame = false;
  *mgvnOptimizing = false;
  mvCalibImgs.clear();
}

void CameraCalibrator::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  ((CameraCalibrator*)ptr)->GUICommandHandler(sCommand, sParams);
}


#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
namespace fs = boost::filesystem;

void conversionCVToNB(cv::Mat frame, Image<byte> &imBW, ATANCamera& mCamera){
	cv::Mat clone = frame.clone();
	cv::Mat_<cv::Vec3b>& frame_p = (cv::Mat_<cv::Vec3b>&)clone;
	for (int i = 0; i < mCamera.GetImageSize()[1]; i++){
		for (int j = 0; j < mCamera.GetImageSize()[0]; j++){	
		imBW[i][j] = (frame_p(i,j)[0] + frame_p(i,j)[1] + frame_p(i,j)[2]) / 3;
		}
	}

}

void CameraCalibrator::GUICommandHandler(string sCommand, string sParams) // Called by the callback func..
{
  if (sCommand == "CameraCalibrator.Reset")
  {
    Reset();
    return;
  };
  if (sCommand == "CameraCalibrator.GrabNextFrame")
  {
    mbGrabNextFrame = true;
    return;
  }
  if (sCommand == "CameraCalibrator.ShowNext")
  {
    int nToShow = (*mgvnShowImage - 1 + 1) % mvCalibImgs.size();
    *mgvnShowImage = nToShow + 1;
    return;
  }
  if (sCommand == "CameraCalibrator.SaveCalib")
  {
    cout << "  Camera calib is " << mCamera.mgvvCameraParams << endl;
    cout << "  Saving camera calib to camera.cfg..." << endl;
    ofstream ofs("camera.cfg");
    if (ofs.good())
    {
      ofs << "Camera.Parameters=[ " << mCamera.mgvvCameraParams << " ]";
      ofs.close();
      cout << "  .. saved." << endl;
    }
    else
    {
      cout << "! Could not open camera.cfg for writing." << endl;
      GV2.PrintVar("Camera.Parameters", cout);
      cout << "  Copy-paste above line to settings.cfg or camera.cfg! " << endl;
    }
    mbDone = true;
  }
if(sCommand=="CameraCalibrator.GrabImages")
    {
      fs::path full_path( fs::initial_path<fs::path>() );
      full_path = fs::system_complete( fs::path( "Images/" ) );

      if ( !fs::exists( full_path ) )
      {
	std::cout << "\nNot found: " << full_path.c_str() << std::endl;
	return;
      }

      if ( fs::is_directory( full_path ) )
      {
	std::cout << "\nIn directory: "
		  << full_path.c_str() << "\n\n";
	fs::directory_iterator end_iter;
	for ( fs::directory_iterator dir_itr( full_path );
	      dir_itr != end_iter;
	      ++dir_itr )
	{
	  try
	  {
	    if ( fs::is_directory( dir_itr->status() ) )
	    {
// 	      ++dir_count;
// 	      std::cout << dir_itr->path().filename() << " [directory]\n";
	    }
	    else if ( fs::is_regular_file( dir_itr->status() ) )
	    {
// 	      ++file_count;
// 	      std::cout << dir_itr->path().filename() << "\n";
	      
	      cv::Mat image;
	      image = cv::imread(dir_itr->path().c_str());
	      printf("path: %s\n", dir_itr->path().c_str());
// 	      cv::Mat grayImage;
// 	      cvtColor(image, grayImage, CV_RGB2GRAY);
// 	      printf("1\n");
	      
	      Image<byte> imBW(mCurrentImage.size());
	      conversionCVToNB(image, imBW, mCamera);
		  
		  // TODO: display 3D points
		  glDrawPixels(imBW);
	      
	      CalibImage c;
	      if(c.MakeFromImage(imBW))
	      {
		printf("Using image %s\n", dir_itr->path().filename().c_str());
		mvCalibImgs.push_back(c);
		mvCalibImgs.back().GuessInitialPose(mCamera);
		
		// TODO: display 3D points
		//mvCalibImgs.back().Draw3DGrid(mCamera, false);
	    mGLWindow->HandlePendingEvents();
	    mGLWindow->swap_buffers();
		//sleep(3);
	      }
	      else
	      {
		printf("NOT using image %s\n", dir_itr->path().filename().c_str());
	      }
	    }
	    else
	    {
// 	      ++other_count;
// 	      std::cout << dir_itr->path().filename() << " [other]\n";
	    }

	  }
	  catch ( const std::exception & ex )
	  {
// 	    ++err_count;
	    std::cout << dir_itr->path().filename().c_str() << " " << ex.what() << std::endl;
	  }
	}
// 	std::cout << "\n" << file_count << " files\n"
// 		  << dir_count << " directories\n"
// 		  << other_count << " others\n"
// 		  << err_count << " errors\n";
      }
      
    }
  if (sCommand == "exit" || sCommand == "quit")
  {
    mbDone = true;
  }
}

void CameraCalibrator::OptimizeOneStep()
{
  int nViews = mvCalibImgs.size();
  int nDim = 6 * nViews + NUMTRACKERCAMPARAMETERS;
  int nCamParamBase = nDim - NUMTRACKERCAMPARAMETERS;
  
  Matrix<> mJTJ(nDim, nDim);
  Vector<> vJTe(nDim);
  mJTJ = Identity; // Weak stabilizing prior
  vJTe = Zeros;

  if(*mgvnDisableDistortion) mCamera.DisableRadialDistortion();

  
  double dSumSquaredError = 0.0;
  int nTotalMeas = 0;
  
  for(int n=0; n<nViews; n++)
    {
      int nMotionBase = n*6;
      vector<CalibImage::ErrorAndJacobians> vEAJ = mvCalibImgs[n].Project(mCamera);

      for(unsigned int i=0; i<vEAJ.size(); i++)
	{
	  CalibImage::ErrorAndJacobians &EAJ = vEAJ[i];
	  // All the below should be +=, but the MSVC compiler doesn't seem to understand that. :(
      mJTJ.slice(nMotionBase, nMotionBase, 6, 6) = 
      mJTJ.slice(nMotionBase, nMotionBase, 6, 6) + EAJ.m26PoseJac.T() * EAJ.m26PoseJac;
      mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) = 
      mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) + EAJ.m2NCameraJac.T() * EAJ.m2NCameraJac;
      mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
      mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
      mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) = 
      mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
      // Above does twice the work it needs to, but who cares..

      vJTe.slice(nMotionBase,6) = 
      vJTe.slice(nMotionBase,6) + EAJ.m26PoseJac.T() * EAJ.v2Error;
      vJTe.slice(nCamParamBase,NUMTRACKERCAMPARAMETERS) = 
      vJTe.slice(nCamParamBase,NUMTRACKERCAMPARAMETERS) + EAJ.m2NCameraJac.T() * EAJ.v2Error;

	  dSumSquaredError += EAJ.v2Error * EAJ.v2Error;
	  ++nTotalMeas;
	}
    };
  
  if(nTotalMeas == 0)
  {
    ROS_WARN_THROTTLE(2, "No new measurements, this can happen for wide angle cameras when \"Camera.Project()\" gets invalid");
    return;
  }

  double lastPixelError = mdMeanPixelError;
  mdMeanPixelError = sqrt(dSumSquaredError / nTotalMeas);

  if(std::abs(lastPixelError - mdMeanPixelError) < 1e-6)
    mDoOptimize = false;

  SVD<> svd(mJTJ);
  Vector<> vUpdate(nDim);
  vUpdate= svd.backsub(vJTe);
  vUpdate *= 0.1; // Slow down because highly nonlinear...
  for(int n=0; n<nViews; n++)
    mvCalibImgs[n].mse3CamFromWorld = SE3<>::exp(vUpdate.slice(n * 6, 6)) * mvCalibImgs[n].mse3CamFromWorld;
  mCamera.UpdateParams(vUpdate.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS));
};















