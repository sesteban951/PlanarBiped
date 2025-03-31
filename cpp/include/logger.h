#ifndef logger_h
#define logger_h

#include <Eigen/Dense>

#include "stdio.h"
#include <iostream>
#include <fstream>

#include <ctime>
#include <iomanip>
#include <sstream>

#include <cstdlib>

class Logger
{
    public: Logger(std::string file_name);

    public: ~Logger();

    public: void AddLabels(std::string labels);

    public: void WriteToLog(Eigen::Vector<double, Eigen::Dynamic>  data);

    public: void OpenFile(std::string file_name);

    public: void CloseFile();

    private: std::ofstream file_id_;
};

#endif