#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>
#include <map>

#include "KeyFrame.h"
#include "Frame.h"
#include "Vocabulary.h"


#include<mutex>


namespace DXSLAM
{

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const Vocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);
    std::vector<KeyFrame*> DetectRelocalizationByglb(Frame* F);

protected:

  // Associated vocabulary
  const Vocabulary* mpVoc;

  // Inverted file
  std::map<int,std::list<KeyFrame*>>mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
