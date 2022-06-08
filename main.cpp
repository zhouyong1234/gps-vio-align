#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

double StringToDouble(std::string strData)
{
    return std::stod(strData);
}


std::vector<std::string> StringSplit(std::string str,std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str+=pattern;
  int size=str.size();
 
  for(int i=0; i<size; i++)
  {
    pos=str.find(pattern,i);
    if(pos<size)
    {
      std::string s=str.substr(i,pos-i);
      result.push_back(s);
      i=pos+pattern.size()-1;
    }
  }
  return result;
}



int main() {


  // std::cout << "------------------------------------------------------------" << std::endl;


  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_tgt(3, 525);
  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_src(3, 525);

  Eigen::Vector3d ypr;

  std::fstream gpsFile("/home/touchair/gps-vio-align/gps.txt");
  std::fstream stateFile("/home/touchair/gps-vio-align/state.txt");
  std::string readLine;

  int i = 0;

  while(std::getline(gpsFile, readLine))
  {
    // std::cout << readLine << std::endl;
    std::vector<std::string> lineSplit = StringSplit(readLine, " ");

    // std::cout << lineSplit[0] << ", " << lineSplit[1] << ", " << lineSplit[2] << std::endl;

    cloud_src(0, i) = StringToDouble(lineSplit[0]);
    cloud_src(1, i) = StringToDouble(lineSplit[1]);
    cloud_src(2, i) = StringToDouble(lineSplit[2]);

    i++;

  }

  int j = 0;

  while(std::getline(stateFile, readLine))
  {
    // std::cout << readLine << std::endl;
    std::vector<std::string> lineSplit = StringSplit(readLine, " ");

    // std::cout << lineSplit[0] << ", " << lineSplit[1] << ", " << lineSplit[2] << std::endl;

    cloud_tgt(0, j) = StringToDouble(lineSplit[0]);
    cloud_tgt(1, j) = StringToDouble(lineSplit[1]);
    cloud_tgt(2, j) = StringToDouble(lineSplit[2]);

    j++;

  }


  Eigen::Matrix4d st = Eigen::umeyama(cloud_src, cloud_tgt, false);

  Eigen::Matrix4d ts = Eigen::umeyama(cloud_tgt, cloud_src, false);



  std::cout << "------------------------------------------------------------" << std::endl;

  std::cout << "-------------------------GPS to VIO-------------------------" << std::endl;

  std::cout << "------------------------------------------------------------" << std::endl;

  std::cout << st << std::endl;

  std::cout << "------------------------------------------------------------" << std::endl;

  ypr = st.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
  std::cout << ypr.transpose() * 180 / M_PI << std::endl;

  std::cout << "------------------------------------------------------------" << std::endl;

  std::cout << "-------------------------VIO to GPS-------------------------" << std::endl;

  std::cout << "------------------------------------------------------------" << std::endl;

  std::cout << ts << std::endl;

  std::cout << "------------------------------------------------------------" << std::endl;

  ypr = ts.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
  std::cout << ypr.transpose() * 180 / M_PI << std::endl;

  std::cout << "------------------------------------------------------------" << std::endl;

  std::cout << st.block<3, 3>(0, 0) * ts.block<3, 3>(0, 0) << std::endl;

  return 0;
}