#include <sstream>
#include <boost/thread.hpp>

#include "convert_octomap.hpp"
#include <pronto_utils/pronto_vis.hpp>
#include <pcl/io/ply_io.h>

#include <ConciseArgs>

struct AppConfig
{
  bool verbose;    
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, ConvertOctomapConfig co_cfg_,
        AppConfig app_cfg);
    
    ~App(){
    }        
    
    ConvertOctomapConfig co_cfg_;
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    ConvertOctomap* convert_;
    
    pronto_vis* pronto_vis_ ;
     
    void doFileProcessing(std::string ply_filename);

  private:

};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, ConvertOctomapConfig co_cfg_,
         AppConfig app_cfg_) : lcm_(lcm_), 
         co_cfg_(co_cfg_), 
         app_cfg_(app_cfg_){
  convert_ = new ConvertOctomap(lcm_, co_cfg_);
  pronto_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );

}

void App::doFileProcessing(std::string ply_filename){
  std::cout << "ply_filename: " << ply_filename << "\n";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  /*
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (ply_filename, *pcl_cloud) == -1){ //* load the file
          std::cout << "Couldn't read pcd file\n";
    exit(-1);
  }      */

  if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (ply_filename, *pcl_cloud) == -1){ //* load the file
          std::cout << "Couldn't read ply file\n";
    exit(-1);
  }

  std::cout << "Started doProcessing\n";
  pronto::PointCloud* cloud (new pronto::PointCloud ());
  pronto::PointCloud* sub_cloud (new pronto::PointCloud ());
  pronto_vis_->convertCloudPclToPronto(*pcl_cloud, *cloud);


  convert_->doWork(cloud);
  convert_->publishOctree( convert_->getTree(),"OCTOMAP");
  exit(-1);
}


int main(int argc, char ** argv) {
  ConvertOctomapConfig co_cfg;
  co_cfg.octomap_resolution = 0.1; // was always 0.1 for mav and atlas
  co_cfg.blur_sigma = 0.1; // default was .5
  co_cfg.blur_map = false;
  AppConfig app_cfg;
  
  std::stringstream s;
  s <<  getDataPath() <<   "/octomap.ply" ;
  std::string ply_filename = s.str();
  int input = 0; // 0 = lcm | 1 = file
  
  ConciseArgs opt(argc, (char**)argv);
  //
  opt.add(co_cfg.octomap_resolution, "R", "octomap_resolution","Resolution of underlying octomap");
  opt.add(co_cfg.blur_sigma, "b", "blur_sigma","Radius of the blur kernel");
  opt.add(co_cfg.blur_map, "u", "blur_map","Blur Map");
  //
  opt.add(ply_filename, "f", "ply_filename","Process this PCD file");    
  opt.add(input, "i", "input","Input mode: 0=lcm 1=file 2=republish pcd only");    
  opt.parse();  
  
  std::cout << "Blur sigma: " << co_cfg.blur_sigma << "\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App* app= new App(lcm, co_cfg, app_cfg);
  
  app->doFileProcessing(ply_filename);

  return 0;
}