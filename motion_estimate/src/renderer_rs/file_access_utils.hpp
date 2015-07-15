#ifndef FILE_ACCESS_UTILS_HPP
#define FILE_ACCESS_UTILS_HPP

#include <iostream>
#include <errno.h>
#include <dirent.h>

using namespace std;


namespace visualization_utils
{

//===============================================================================
// FILE ACCESS
  

      
  // this function should go into otdf_utils library
  inline static int get_OTDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if(fn.substr(fn.find_last_of(".") + 1) == "otdf") 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      std::sort( files.begin(), files.end() ); // sort file names alphabetically
      closedir(dp);
      return 0;
  }
   //-------------------------------------------------------------------------------
  // this function should go into otdf_utils library
  inline static int get_URDF_or_SDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if((fn.substr(fn.find_last_of(".") + 1) == "urdf")||(fn.substr(fn.find_last_of(".") + 1) == "sdf")) 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      std::sort( files.begin(), files.end() ); // sort file names alphabetically
      closedir(dp);
      return 0;
  }
  
  inline static int get_URDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if(fn.substr(fn.find_last_of(".") + 1) == "urdf") 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      std::sort( files.begin(), files.end() ); // sort file names alphabetically
      closedir(dp);
      return 0;
  } 
  
   

}

#endif
