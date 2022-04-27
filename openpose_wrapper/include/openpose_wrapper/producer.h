#ifndef open_ptrack_opw_producer_h
#define open_ptrack_opw_producer_h

// Internal dependencies
#include "openpose_wrapper/commons/commons.h"

namespace open_ptrack {
  namespace opw {

    //================================
    // OpenPoseWrapper: Producer Class
    //================================
    class Producer : public commons::OpenPoseProducer
    {
    public:
      //---------------------------
      // Constructors / Destructors
      //---------------------------
      Producer();
      ~Producer() override;

      //----------------------------------
      // Public virtual functions override
      //----------------------------------
      void initializationOnThread() override;

      //------------------------
      // Custom public functions
      //------------------------
      void setImage(const op::Matrix& t_image);
      commons::framesDetectionsSharedPtr run();

    private:
      //-----------------------------------
      // Private virtual functions override
      //-----------------------------------
      commons::framesDetectionsSharedPtr workProducer() override;

      //-------------------------
      // Private member variables
      //-------------------------
      std::shared_ptr<const op::Matrix> m_img;
    };

  } // namespace opw
} // namespace open_ptrack

#endif
