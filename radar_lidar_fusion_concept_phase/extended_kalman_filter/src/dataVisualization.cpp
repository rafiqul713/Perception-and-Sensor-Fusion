#include "../header/matplotlibcpp.h"
#include <iostream>
#include<fstream>
#include<string>
#include <sstream>
#include <vector>
namespace plt = matplotlibcpp;

using namespace std;

int main() {

    string line;
    string a, b, c,d,e,f,g,h;
    ifstream inputfile("C:/Users/Asif/Desktop/Sensor-Fusion/radar_lidar_fusion_concept_phase/extended_kalman_filter/sensor_data/1.txt");
    vector<int> lider;
    vector<int> rader;
    vector<int> lider2;
    vector<int> rader2;
    //Defining the loop for getting input from the file

    while (getline(inputfile, line)) {
        istringstream ss(line);
        if (ss >> a >> b >> c >> d >> e >> f >> g >> h)
        {
         
             lider.push_back(stod(b));
             rader.push_back(stod(c));
             lider2.push_back(stod(g));
             rader2.push_back(stod(h));
        }
    }
  
   plt::figure_size(1500, 980);
   plt::title("Rader lider data"); // add a title
   plt::plot(lider, rader, "g--");
   plt::plot(lider2, rader2,"r--");
 //  plt::plot(lider2, rader2, "r--");
   plt::xlabel("lider");
   plt::ylabel("rader");
   plt::xlim(-1, 20);
   plt::ylim(-1,20);
   plt::show();


}