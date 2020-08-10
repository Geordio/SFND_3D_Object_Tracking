#pragma once
 
#include <iostream>
#include <mutex>
#include <fstream>

using namespace std;


  class Logger
  {
  public:
    // ctor
    Logger(std::string fileName);
 
    ~Logger()     
    {
      fileStream.close();
    }
 
    void WriteLine(std::string content);
 
  private:
    std::ofstream fileStream;
    std::mutex mMutex;
  };


// #include <string>

// class Logger{
// public:
//    static Logger* Instance();
//    bool openLogFile(std::string logFile);
//    void writeToLogFile();
//    bool closeLogFile();

// private:
//    Logger(){};  // Private so that it can  not be called
//    Logger(Logger const&){};             // copy constructor is private
//    Logger& operator=(Logger const&){};  // assignment operator is private
//    static Logger* m_pInstance;
// };
