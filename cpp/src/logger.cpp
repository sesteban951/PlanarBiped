#include "logger.h"

Logger::Logger(std::string file_name)
{
    this->OpenFile(file_name);
}

void Logger::AddLabels(std::string labels)
{
    this->file_id_ << labels << std::endl;
}

Logger::~Logger()
{
    this->file_id_.close();
}

void Logger::OpenFile(std::string file_name)
{
    this->file_id_.open(file_name);

    if(!this->file_id_)
    {
        std::cerr << "Failed to open text file!" << std::endl;
    }
}

void Logger::WriteToLog(Eigen::Vector<double, Eigen::Dynamic> data)
{
    int n = data.rows();

    for(int i = 0; i < n; i++)
    {
        this->file_id_ << data(i) << ",";
    }
    this->file_id_ << "\n";
}

void Logger::CloseFile()
{
    this->file_id_.close();
}