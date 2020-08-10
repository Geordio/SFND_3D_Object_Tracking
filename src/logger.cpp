 #include "logger.h"

  Logger::Logger(std::string fileName)
  {
    fileStream.open(fileName);
 
    if (fileStream.fail())
    {
      throw std::iostream::failure("Cannot open file: " + fileName);
    }
  }  
 
  void Logger::WriteLine(std::string text)
  {
    fileStream << text << std::endl;
  }
