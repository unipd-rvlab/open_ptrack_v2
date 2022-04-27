// Standard dependencies
#include <iostream>

// Internal dependencies
#include "openpose_wrapper/producer.h"

//---------------------------
// Constructors / Destructors
//---------------------------
// Constructor implementation
open_ptrack::opw::Producer::Producer()
  : m_img(nullptr){};

// Virtual destructor override implementation
open_ptrack::opw::Producer::~Producer(){};

//-------------------------------------------
// Inherited virtual functions implementation
//-------------------------------------------
// Produce OpenPose input from last received image
open_ptrack::opw::commons::framesDetectionsSharedPtr open_ptrack::opw::Producer::workProducer()
{
  // If there are no image to process notify and move forward
  if (!m_img) {
    std::cout << "No image to process. Moving forward" << std::endl;
    this->stop();
    return nullptr;
  }

  // Create OpenPose input structure and fill it
  opw::commons::framesDetectionsSharedPtr detections_vector_ptr =
    std::make_shared<opw::commons::frameDetectionsVector>(0);

  detections_vector_ptr->emplace_back(std::make_shared<op::Datum>());
  m_img->copyTo(detections_vector_ptr->front()->cvInputData);

  // Decrement shared pointer counter, if last one clear it
  m_img.reset();

  return detections_vector_ptr;
}

void open_ptrack::opw::Producer::initializationOnThread(){};

//--------------------------------
// Custom functions implementation
//--------------------------------
// Public interface to produce OpenPose input data from last received image
open_ptrack::opw::commons::framesDetectionsSharedPtr open_ptrack::opw::Producer::run()
{
  return workProducer();
}

// Update image
void open_ptrack::opw::Producer::setImage(const op::Matrix& t_image)
{
  m_img = std::make_shared<const op::Matrix>(t_image);
}
