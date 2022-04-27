#ifndef open_ptrack_opw_commons_h
#define open_ptrack_opw_commons_h

// Standard dependencies
#include <map>
#include <memory>
#include <vector>

// OpenPose dependencies
#include <openpose/core/datum.hpp>
#include <openpose/pose/poseParameters.hpp>
#include <openpose/thread/workerProducer.hpp>

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

//==================================================
// OpenPoseWrapper Convenience Definitions (Commons)
//==================================================
namespace open_ptrack {
  namespace opw {
    namespace commons {

      enum OpenPoseModelType
      {
        BODY_25 = 0,
        COCO_18 = 1,
        MPI_15 = 2,
        MPI_4_LAYERS_15 = 3,
        R_HAND_21 = 4,
        L_HAND_21 = 5,
        FACE_70 = 6
      };

      static const std::map<std::string, OpenPoseModelType> model_name_to_type = {{"BODY_25", BODY_25},
                                                                                  {"COCO", COCO_18},
                                                                                  {"MPI", MPI_15},
                                                                                  {"MPI_4_layers", MPI_4_LAYERS_15},
                                                                                  {"HAND_r", R_HAND_21},
                                                                                  {"HAND_l", L_HAND_21},
                                                                                  {"FACE", FACE_70}};

      //----------------------------------------------
      // Common utilities to increase code readability
      //----------------------------------------------
      using frameDetectionsSharedPtr = std::shared_ptr<op::Datum>;
      using frameDetectionsVector = std::vector<frameDetectionsSharedPtr>;
      using framesDetectionsSharedPtr = std::shared_ptr<frameDetectionsVector>;

      using OpenPoseProducer = op::WorkerProducer<framesDetectionsSharedPtr>;

      using keypointsIdToName = std::map<unsigned int, std::string>;
      using modelToKeypointNames = std::map<OpenPoseModelType, keypointsIdToName>;

      static const modelToKeypointNames keypoint_names_from_model = {
        {OpenPoseModelType::BODY_25, op::getPoseBodyPartMapping(op::PoseModel::BODY_25)},
        {OpenPoseModelType::COCO_18, op::getPoseBodyPartMapping(op::PoseModel::COCO_18)},
        {OpenPoseModelType::MPI_15, op::getPoseBodyPartMapping(op::PoseModel::MPI_15)},
        {OpenPoseModelType::MPI_4_LAYERS_15, op::getPoseBodyPartMapping(op::PoseModel::MPI_15_4)}};

      template <typename ContainerT, typename PredicateT>
      void erase_if(ContainerT& items, const PredicateT& predicate)
      {
        for (auto it = items.begin(); it != items.end();) {
          if (predicate(*it)) {
            it = items.erase(it);
          }
          else {
            ++it;
          }
        }
      }

    } // namespace commons
  } // namespace opw
} // namespace open_ptrack

#endif
