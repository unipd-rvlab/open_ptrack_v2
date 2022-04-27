#ifndef open_ptrack_opw_consumer_h
#define open_ptrack_opw_consumer_h

// Internal dependencies
#include "openpose_wrapper/commons/commons.h"

// Custom External Packages dependencies
#include "skeletons/types.h"

namespace open_ptrack {
  namespace opw {

    const int OP_DATUM_PERSON_DIMENSION = 0;
    const int OP_DATUM_PARTS_DIMENSION = 1;
    const int OP_DATUM_LAST_DIMENSION_SIZE = 3;
    const int OP_KP_X = 0;
    const int OP_KP_Y = 1;
    const int OP_KP_CONFIDENCE = 2;
    const int OP_LEFT_HAND = 0;
    const int OP_RIGHT_HAND = 1;
    const int NR_OF_PEOPLE_ERROR = -1;

    //================================
    // OpenPoseWrapper: Consumer Class
    //================================
    class Consumer // it is async, no need to inherit from openpose WorkerConsumer class
    {
      // This class is actually needed since I don't want that the wrapper itself mess with
      // the dirty openpose data strucure The only thing this class is in charge is to parse
      // the op datum into the skeletonGroup structure, then the wrapper will call the
      // proper functions to make that structure as ros messages.

    public:
      //---------------------------
      // Constructors / Destructors
      //---------------------------
      Consumer();
      ~Consumer();

      //------------------------
      // Custom public functions
      //------------------------
      void setInputData(commons::framesDetectionsSharedPtr& t_inputData);

      std::unique_ptr<skeletons::types::SkeletonGroup> processData();
      void configure(const bool t_body_enabled,
                     const std::string t_body_model_name,
                     const bool t_hands_enabled,
                     const std::string t_hands_model_name,
                     const bool t_face_enabled,
                     const std::string t_face_model_name);

    private:
      //-------------------
      // Private structs
      //-------------------
      struct OpwParameters
      {
        bool body_enabled;
        std::string body_model_name;

        bool hands_enabled;
        std::string hand_model_name;

        bool face_enabled;
        std::string face_model_name;
      };

      //------------------
      // Private functions
      //------------------
      void clearInputData();
      int checkAndGetNumberOfPeople();
      int getNumberOfPeople();

      //-------------------------
      // Private member variables
      //-------------------------
      // Openpose Datum
      commons::framesDetectionsSharedPtr m_input_data;
      // Parameters struct
      OpwParameters m_op_configuration;
    };

  } // namespace opw
} // namespace open_ptrack

#endif
