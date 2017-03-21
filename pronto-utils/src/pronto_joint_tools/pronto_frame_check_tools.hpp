#ifndef __PRONTO_FRAME_CHECK_TOOLS_HPP__
#define __PRONTO_FRAME_CHECK_TOOLS_HPP__

class FrameCheckTools
{
public:
  FrameCheckTools (){
    //local_to_head_frame_valid_ = false;

  };
  ~FrameCheckTools() {}
  

  // Get the list of links between to frames e.g. from local to camera
  std::vector < std::pair<std::string,std::string> > getPathBetweenFrames(BotFrames *botframes, 
        std::string from_frame, std::string final_to_frame){
    std::vector < std::pair<std::string,std::string> > chain;

    while( final_to_frame != std::string( bot_frames_get_relative_to(botframes,from_frame.c_str()) ) ) {

      std::string to_frame = std::string( bot_frames_get_relative_to(botframes,from_frame.c_str())) ;
      // std::cout << "> " << from_frame << " to " << to_frame << "\n";

      std::pair <std::string,std::string> link (from_frame,to_frame);   // value init
      chain.push_back(link);
      from_frame = to_frame;
    }
    std::string to_frame = std::string( bot_frames_get_relative_to(botframes,from_frame.c_str())) ;
    std::pair <std::string,std::string> link (from_frame,to_frame);   // value init
    chain.push_back(link);

    //std::cout << "> " << from_frame << " to " << to_frame << " (last)\n";
    //std::cout << "done\n";
  
    return chain;
  }

  // Determine of all of the dynamic/updated frames between from_frame and to_frame
  // have in fact been updated more recently than current_utime
  // This is an explicit check that all frames are up-to-date
  bool isFrameValid(BotFrames* botframes, BotParam* botparam, 
        std::string from_frame, std::string to_frame, int64_t current_utime){

    std::vector < std::pair<std::string,std::string> > chain;
    chain = getPathBetweenFrames(botframes, from_frame, to_frame);

    for (size_t i=0 ; i< chain.size() ; i++){
      std::string key = std::string("coordinate_frames.") + chain[i].first + std::string(".updated") ;
      bool updated = bot_param_get_boolean_or_fail(botparam, key.c_str() );
      //std::cout << chain[i].first << " to " << chain[i].second << " "
      //          << (updated ? "UPDATED" : "NOT UPDATED")  << " during operation\n";

      if (updated){
        int64_t last_update_utime;
        bot_frames_get_trans_latest_timestamp(botframes, 
             chain[i].first.c_str(), chain[i].second.c_str(), &last_update_utime);
        if (current_utime > last_update_utime){
          std::cout << "Invalid Chain: "<< chain[i].first << " to " << chain[i].second 
             << " is "
             << std::setprecision(16)
             << ((double) (last_update_utime)*1e-6) 
             <<" which older than request "
             << ((double) (current_utime)*1e-6) 
             << std::setprecision(6)
             << "\n";
          false;
        }
      }

    }
    return true;
  }


  // Hard-coded channel names is depreciated - now explictly check the required frames:
  /*
  // Check if the SCAN frames have been updated at all.
  bool isLocalToScanValid(BotFrames* botframes){
    std::cout << "isLocalToScanValid\n";

    if (local_to_head_frame_valid_){
      return true; 
    }else{ // check each updating frame has been updated already.      
      int64_t last_update_utime_1, last_update_utime_2, last_update_utime_3;

      if (!bot_frames_get_trans_latest_timestamp(botframes, "PRE_SPINDLE", "POST_SPINDLE", &last_update_utime_1)){
        std::cout << "p2p is invalid\n";
        return false; 
      }
      if (!bot_frames_get_trans_latest_timestamp(botframes, "head", "body", &last_update_utime_2)){
        std::cout << "h2b is invalid\n";
        return false; 
      }
      if (!bot_frames_get_trans_latest_timestamp(botframes, "body", "local", &last_update_utime_3)){
        std::cout << "b2l is invalid\n";
        return false; 
      }
      
      std::cout << "PRE_SPINDLE" << " to " << "POST_SPINDLE" << " last updated " << last_update_utime_1 << "\n";
      std::cout << "head" << " to " << "body" << " last updated " << last_update_utime_2 << "\n";
      std::cout << "body" << " to " << "local" << " last updated " << last_update_utime_3 << "\n";
      
//      if (( (last_update_utime_1 > 0) && (last_update_utime_2 > 0) )  && (last_update_utime_3 > 0) ){
      if ( (last_update_utime_1 > 0)   && (last_update_utime_3 > 0) ){
        local_to_head_frame_valid_ = true;
        return true;
      }else{
        return false;
      }
    }
  }
  
  
  // Check if the SCAN frames are more recent than the scan utime (to avoid extrapolation):
  bool isLocalToScanUpdated(BotFrames* botframes, int64_t scan_utime){
    int64_t last_update_utime_1, last_update_utime_2, last_update_utime_3;
    if (!bot_frames_get_trans_latest_timestamp(botframes, "PRE_SPINDLE", "POST_SPINDLE", &last_update_utime_1)){
      return false; 
    }
    if (!bot_frames_get_trans_latest_timestamp(botframes, "head", "body", &last_update_utime_2)){
      return false; 
    }
    if (!bot_frames_get_trans_latest_timestamp(botframes, "body", "local", &last_update_utime_3)){
      return false; 
    }
    
    std::cout << "SCAN " << scan_utime << "\n";
    std::cout << "PRE_SPINDLE" << " to " << "POST_SPINDLE" << " last updated " << last_update_utime_1 << "\n";
    std::cout << "head" << " to " << "body" << " last updated " << last_update_utime_2 << "\n";
    std::cout << "body" << " to " << "local" << " last updated " << last_update_utime_3 << "\n";
    
// one for val and one for hyq
//    if (( (last_update_utime_1 >= scan_utime) && (last_update_utime_2 >= scan_utime) )  && (last_update_utime_3 >= scan_utime) ){
    if (( (last_update_utime_1 >= scan_utime) && (last_update_utime_3 >= scan_utime) )){
      local_to_head_frame_valid_ = true;
      return true;
    }else{
      return false;
    }
  }  
  */
  
private:
  
  //bool local_to_head_frame_valid_;
};


#endif
