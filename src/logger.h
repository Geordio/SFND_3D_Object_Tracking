#pragma once
 
#include <iostream>
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
 
    void WriteLine(std::string text);
 
  private:
    std::ofstream fileStream;
  };
