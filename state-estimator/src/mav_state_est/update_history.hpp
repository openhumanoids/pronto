#ifndef __update_history_h__
#define __update_history_h__

#include <map>
#include <stl_utils/stlmap_utils.hpp>
#include <stdint.h>

#include "rbis_update_interface.hpp"

namespace MavStateEst {

class updateHistory {
public:

  typedef std::multimap<int64_t, RBISUpdateInterface *> historyMap;
  typedef historyMap::iterator historyMapIterator;
  typedef std::pair<int64_t, RBISUpdateInterface*> historyPair;
  historyMap updateMap;

  updateHistory(RBISUpdateInterface * init);
  ~updateHistory();

  /**
   * @brief removeCollectionFromHistory removes a collection of objects from the history
   * @param rbisus vector of objects to be removed
   * @return a pointer to the element before the first in the collection
   */
  historyMapIterator removeSeriesFromHistory(std::vector<RBISUpdateInterface*> rbisus);

  /**
   * @brief removeFromHistory removes an object from history
   * @param rbisu the object to be removed
   * @return a pointer to the object before the one just removed
   */
  historyMapIterator removeFromHistory(RBISUpdateInterface* rbisu);

  /**
   * add object to history
   *
   * returns iterator to obj in the history map
   */
  historyMapIterator addToHistory(RBISUpdateInterface * rbisu);

  /**
   * Clear everything in the history older than the given timestamp
   */
  void clearHistoryBeforeUtime(int64_t utime);

};

}

#endif
