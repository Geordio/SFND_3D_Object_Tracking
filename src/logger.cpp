
 #include "logger.h"

  Logger::Logger(std::string fileName)
  {
    fileStream.open(fileName);
 
    if (fileStream.fail())
    {
      throw std::iostream::failure("Cannot open file: " + fileName);
    }
  }  
 
  void Logger::WriteLine(std::string content)
  {
    // std::lock_guard lock(mMutex);
     
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char timestamp[26];
    // ctime_s(timestamp, sizeof timestamp, &now);
 
    std::string timestampWithoutEndl(timestamp);    
    timestampWithoutEndl = timestampWithoutEndl.substr(0, 24);
 
    fileStream << timestampWithoutEndl << ": " << content << std::endl;

    cout << "WRTTEN"<< endl;
  }
