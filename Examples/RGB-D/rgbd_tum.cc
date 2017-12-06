/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
// rectified by zzh in 2017.

//#define TRACK_WITH_IMU_VQMAW//we use the last parameter as the odometryData's number, default is 15

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

ORB_SLAM2::System* g_pSLAM;
double g_simulateTimestamp=-1;
bool g_brgbdFinished=false;
mutex g_mutex;

//a new thread simulating the odom serial threads
void odomRun(ifstream &finOdomdata,int totalNum){//must use &
  //read until reading over
  int nTotalNum=2+4+3*3;
  if (totalNum!=0) nTotalNum=totalNum;
  double* odomdata=new double[nTotalNum];
  double timestamp,tmstpLast=-1;
  
  while (!g_pSLAM){//if it's NULL
    sleep(0.1);//wait 0.1s
  }
  while (!finOdomdata.eof()){
    finOdomdata>>timestamp;
    if (finOdomdata.eof())
      break;
    while (1){//until the image reading time is reached
      {
      unique_lock<mutex> lock(g_mutex);
      if (timestamp<=g_simulateTimestamp||g_brgbdFinished)
	break;
      }
      usleep(15000);//allow 15ms delay
    }
    for (int i=0;i<nTotalNum;++i){
      finOdomdata>>odomdata[i];
    }
    if (timestamp>tmstpLast)//avoid == condition
#ifndef TRACK_WITH_IMU
      g_pSLAM->TrackOdom(timestamp,odomdata,(char)ORB_SLAM2::System::BOTH);
#else
      g_pSLAM->TrackOdom(timestamp,odomdata+(nTotalNum-6),(char)ORB_SLAM2::System::IMU);//jump vl,vr,quat[4],magnetic data[3] then it's axyz,wxyz for default 15, please ensure the last 6 data is axyz,wxyz
#endif
    //cout<<green<<timestamp<<white<<endl;
    tmstpLast=timestamp;
  }
  delete []odomdata;
  finOdomdata.close();
  cout<<green"Simulation of Odom Data Reading is over."<<white<<endl;
}

int main(int argc, char **argv)
{
    thread* pOdomThread=NULL;
    ifstream finOdomdata;
    int totalNum=0;
    cout<<fixed<<setprecision(6)<<endl;
  
    switch (argc){
      case 5:
	break;
      case 7:
	totalNum=atoi(argv[6]);
      case 6:
	{
	finOdomdata.open(argv[5]);
	if (!finOdomdata.is_open()){
	  cerr<< red"Please check the last path_to_odometryData"<<endl;
	  return -1;
	}
        string strTmp;
	getline(finOdomdata,strTmp);getline(finOdomdata,strTmp);getline(finOdomdata,strTmp);//odom.txt should have 3 unused lines
        pOdomThread=new thread(&odomRun,ref(finOdomdata),totalNum);//must use ref()
	}
	break;
      default:
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
	cerr << red"Or: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_odometryData (number of odometryData)"<<endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    g_pSLAM=&SLAM;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
	{
	unique_lock<mutex> lock(g_mutex);
	g_simulateTimestamp=tframe;//update g_simulateTimestamp
	}

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

	//double data[9]={0.5,0.4};
	//SLAM.SaveFrame("./test_save/",imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    {
    unique_lock<mutex> lock(g_mutex);
    g_brgbdFinished=true;
    }

    // Stop all threads
    if (SLAM.MapChanged()){
      cout<<"Map is changing!Please enter s to stop!"<<endl;
      while (cin.get()!='s') {sleep(1);}
    }
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt"); 
    SLAM.SaveMap("Map.pcd");
    
    //wait for pOdomThread finished
    if (pOdomThread!=NULL)
      pOdomThread->join();

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t,t2;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t2;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
	    /*char ch[30];//at least 22+6=28
	    sprintf(ch,"depth/%.6f.png",t);
	    vstrImageFilenamesD.push_back(ch);*/
        }
    }
}
