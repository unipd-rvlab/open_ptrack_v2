// Standard dependencies
#include <algorithm>

// OpenPose dependencies
#include <openpose/core/array.hpp>

// Custom external dependencies
#include "openpose_wrapper/skeletons/utils.h"

// Internal dependencies
#include "openpose_wrapper/consumer.h"

//================================
// OpenPoseWrapper: Consumer Class
//================================

//---------------------------
// Constructors / Destructors
//---------------------------
// Constructor implementation
open_ptrack::opw::Consumer::Consumer()
  : m_input_data(nullptr){};

// Destructor implementation
open_ptrack::opw::Consumer::~Consumer(){};

//--------------------------------
// Custom functions implementation
//--------------------------------
void open_ptrack::opw::Consumer::setInputData(open_ptrack::opw::commons::framesDetectionsSharedPtr& t_inputData)
{
  m_input_data = t_inputData;
}

void open_ptrack::opw::Consumer::clearInputData()
{
  m_input_data.reset();
}

void open_ptrack::opw::Consumer::configure(const bool t_body_enabled,
                                     const std::string t_body_model_name,
                                     const bool t_hands_enabled,
                                     const std::string t_hands_model_name,
                                     const bool t_face_enabled,
                                     const std::string t_face_model_name)
{
  m_op_configuration = {
    t_body_enabled, t_body_model_name, t_hands_enabled, t_hands_model_name, t_face_enabled, t_face_model_name};
}

int open_ptrack::opw::Consumer::checkAndGetNumberOfPeople()
{
  auto& op_bodies = m_input_data->front()->poseKeypoints;
  auto& op_hands = m_input_data->front()->handKeypoints;
  auto& op_faces = m_input_data->front()->faceKeypoints;

  int n_people = NR_OF_PEOPLE_ERROR;

  if (m_op_configuration.body_enabled) {
    n_people = op_bodies.getSize(OP_DATUM_PERSON_DIMENSION);
    if (m_op_configuration.hands_enabled) {
      if (n_people != op_hands[OP_LEFT_HAND].getSize(OP_DATUM_PERSON_DIMENSION)
          || n_people != op_hands[OP_RIGHT_HAND].getSize(OP_DATUM_PERSON_DIMENSION)
          || op_hands[OP_LEFT_HAND].getSize(OP_DATUM_PERSON_DIMENSION)
               != op_hands[OP_RIGHT_HAND].getSize(OP_DATUM_PERSON_DIMENSION)) {
        return NR_OF_PEOPLE_ERROR;
      }
      else if (m_op_configuration.face_enabled) {
        if (n_people != op_faces.getSize(OP_DATUM_PERSON_DIMENSION)) {
          return NR_OF_PEOPLE_ERROR;
        }
      }
    }
    else if (!m_op_configuration.hands_enabled && m_op_configuration.face_enabled) {
      if (n_people != op_faces.getSize(OP_DATUM_PERSON_DIMENSION)) {
        return NR_OF_PEOPLE_ERROR;
      }
    }
  }
  else if (m_op_configuration.hands_enabled) {
    if (op_hands[OP_LEFT_HAND].getSize(OP_DATUM_PERSON_DIMENSION)
        != op_hands[OP_RIGHT_HAND].getSize(OP_DATUM_PERSON_DIMENSION)) {
      return NR_OF_PEOPLE_ERROR;
    }
    else {
      n_people = op_hands[OP_LEFT_HAND].getSize(OP_DATUM_PERSON_DIMENSION);
      if (m_op_configuration.face_enabled) {
        if (n_people != op_faces.getSize(OP_DATUM_PERSON_DIMENSION)) {
          return NR_OF_PEOPLE_ERROR;
        }
      }
    }
  }
  else if (m_op_configuration.face_enabled) {
    n_people = op_faces.getSize(OP_DATUM_PERSON_DIMENSION);
  }
  else {
    // Not going to happen, shutting down before
    return NR_OF_PEOPLE_ERROR;
  }

  return n_people;
};

int open_ptrack::opw::Consumer::getNumberOfPeople()
{
  auto& op_bodies = m_input_data->front()->poseKeypoints;
  auto& op_hands = m_input_data->front()->handKeypoints;
  auto& op_faces = m_input_data->front()->faceKeypoints;

  int n_people = 0;
  if (m_op_configuration.body_enabled) {
    n_people = op_bodies.getSize(OP_DATUM_PERSON_DIMENSION);
  }
  else if (m_op_configuration.hands_enabled) {
    n_people = op_hands[OP_LEFT_HAND].getSize(OP_DATUM_PERSON_DIMENSION);
  }
  else if (m_op_configuration.face_enabled) {
    n_people = op_faces.getSize(OP_DATUM_PERSON_DIMENSION);
  }
  else {
    // Not going to happen, shutting down before
    return NR_OF_PEOPLE_ERROR;
  }

  return n_people;
};

std::unique_ptr<open_ptrack::skeletons::types::SkeletonGroup> open_ptrack::opw::Consumer::processData()
{
  if (!m_input_data) {
    std::cout << "Nothing to process" << std::endl;
    return nullptr;
  }

  const int n_people = getNumberOfPeople();
  if (n_people == NR_OF_PEOPLE_ERROR) {
    std::cout << "Number of people inconsistent among detectors" << std::endl;
    return nullptr;
  }

  auto& op_bodies = m_input_data->front()->poseKeypoints;
  auto& op_hands = m_input_data->front()->handKeypoints;
  auto& op_faces = m_input_data->front()->faceKeypoints;

  std::unique_ptr<open_ptrack::skeletons::types::SkeletonGroup> out{new open_ptrack::skeletons::types::SkeletonGroup};

  for (int person = 0; person < n_people; ++person) {
    // Since person IDs are randomly assigned by OpenPose, for consistency better not to write anything
    skeletons::types::Skeleton sk;

    if (m_op_configuration.body_enabled) {
      sk.confidence = static_cast<double>(m_input_data->front()->poseScores.at(person));

      auto model_id = opw::commons::model_name_to_type.at(m_op_configuration.body_model_name);
      auto n_body_markers = op_bodies.getSize(OP_DATUM_PARTS_DIMENSION);
      sk.max_markers += static_cast<unsigned int>(n_body_markers);

      for (int body_part = 0; body_part < n_body_markers; ++body_part) {
        sk.addMarker(skeletons::types::Marker(100 * model_id + body_part,
                                              static_cast<double>(op_bodies[{person, body_part, OP_KP_CONFIDENCE}]),
                                              open_ptrack::skeletons::types::KinematicState(open_ptrack::skeletons::types::Point(
                                                static_cast<double>(op_bodies[{person, body_part, OP_KP_X}]),
                                                static_cast<double>(op_bodies[{person, body_part, OP_KP_Y}]),
                                                std::numeric_limits<double>::quiet_NaN()))));
      }
    }

    if (m_op_configuration.hands_enabled) {
      // Right hand
      auto model_id = opw::commons::model_name_to_type.at((m_op_configuration.hand_model_name + "_r"));
      auto n_rhand_markers = op_hands[OP_RIGHT_HAND].getSize(OP_DATUM_PARTS_DIMENSION);
      sk.max_markers += static_cast<unsigned int>(n_rhand_markers);

      for (int hand_part = 0; hand_part < n_rhand_markers; ++hand_part) {
        sk.addMarker(
          skeletons::types::Marker(100 * model_id + hand_part,
                                   static_cast<double>(op_hands[OP_RIGHT_HAND][{person, hand_part, OP_KP_CONFIDENCE}]),
                                   open_ptrack::skeletons::types::KinematicState(open_ptrack::skeletons::types::Point(
                                     static_cast<double>(op_hands[OP_RIGHT_HAND][{person, hand_part, OP_KP_X}]),
                                     static_cast<double>(op_hands[OP_RIGHT_HAND][{person, hand_part, OP_KP_Y}]),
                                     std::numeric_limits<double>::quiet_NaN()))));
      }

      // Left hand
      model_id = opw::commons::model_name_to_type.at((m_op_configuration.hand_model_name + "_l"));
      auto n_lhand_markers = op_hands[OP_LEFT_HAND].getSize(OP_DATUM_PARTS_DIMENSION);
      sk.max_markers += static_cast<unsigned int>(n_lhand_markers);

      for (int hand_part = 0; hand_part < n_lhand_markers; ++hand_part) {
        sk.addMarker(
          skeletons::types::Marker(100 * model_id + hand_part,
                                   static_cast<double>(op_hands[OP_LEFT_HAND][{person, hand_part, OP_KP_CONFIDENCE}]),
                                   open_ptrack::skeletons::types::KinematicState(open_ptrack::skeletons::types::Point(
                                     static_cast<double>(op_hands[OP_LEFT_HAND][{person, hand_part, OP_KP_X}]),
                                     static_cast<double>(op_hands[OP_LEFT_HAND][{person, hand_part, OP_KP_Y}]),
                                     std::numeric_limits<double>::quiet_NaN()))));
      }
    }

    if (m_op_configuration.face_enabled) {
      auto model_id = opw::commons::model_name_to_type.at(m_op_configuration.face_model_name);
      auto n_face_markers = op_faces.getSize(OP_DATUM_PARTS_DIMENSION);
      sk.max_markers += static_cast<unsigned int>(n_face_markers);

      for (int face_part = 0; face_part < n_face_markers; ++face_part) {
        sk.addMarker(skeletons::types::Marker(100 * model_id + face_part,
                                              static_cast<double>(op_faces[{person, face_part, OP_KP_CONFIDENCE}]),
                                              open_ptrack::skeletons::types::KinematicState(open_ptrack::skeletons::types::Point(
                                                static_cast<double>(op_faces[{person, face_part, OP_KP_X}]),
                                                static_cast<double>(op_faces[{person, face_part, OP_KP_Y}]),
                                                std::numeric_limits<double>::quiet_NaN()))));
      }
    }

    sk.bounding_box = open_ptrack::skeletons::utils::computeBoundingBox(sk);
    out->addSkeleton(sk);
  }

  clearInputData();
  return out;
}
