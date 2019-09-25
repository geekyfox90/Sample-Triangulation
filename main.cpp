/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define USE_FREE
#include <iostream>
#include <string>
#include <vector>

#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpencv_traits.h"

#include "SolARModuleTools_traits.h"

#ifndef USE_FREE
#include "SolARModuleNonFreeOpencv_traits.h"
#endif

#include "xpcf/xpcf.h"
#include "core/Log.h"
#include "api/image/IImageLoader.h"
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "core/Log.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
#ifndef USE_FREE
using namespace SolAR::MODULES::NONFREEOPENCV;
#endif

#ifdef SOLAR_USE_OPENGL
#include "SolARModuleOpengl_traits.h"
using namespace SolAR::MODULES::OPENGL;
#endif

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv){

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
#ifdef USE_FREE
    if(xpcfComponentManager->load("conf_Triangulation.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Triangulation.xml")
        return -1;
    }
#else
    if(xpcfComponentManager->load("conf_Triangulation_nf.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Triangulation_nf.xml")
        return -1;
    }
#endif
    // declare and create components
    LOG_INFO("Start creating components");

    // component declaration and creation
    auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
    LOG_INFO("Camera loaded");

/*
 * RESOLVE ??
 *
 */
    auto imageLoader1 = xpcfComponentManager->create<SolARImageLoaderOpencv>("image1")->bindTo<image::IImageLoader>();
    LOG_INFO("Image 1 loaded");
    auto imageLoader2 = xpcfComponentManager->create<SolARImageLoaderOpencv>("image2")->bindTo<image::IImageLoader>();
    LOG_INFO("Image 2 loaded");
  ///////////////////////////////////
  ///////////////////////////////////

#ifdef USE_FREE
    LOG_INFO("free keypoint detector");
    auto keypointsDetector =xpcfComponentManager->resolve<features::IKeypointDetector>();
#else
    LOG_INFO("nonfree keypoint detector");
    auto  keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
#endif

#ifdef USE_FREE
    LOG_INFO("free keypoint extractor");
    auto descriptorExtractor =xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
#else
    LOG_INFO("nonfree keypoint extractor");
    auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
#endif

    auto matcher =xpcfComponentManager->resolve<features::IDescriptorMatcher>();
    auto overlayMatches =xpcfComponentManager->resolve<display::IMatchesOverlay>();
    auto viewerMatches =xpcfComponentManager->resolve<display::IImageViewer>();
    auto poseFinderFrom2D2D =xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D2D>();
    auto triangulator =xpcfComponentManager->resolve<solver::map::ITriangulator>();
    auto mapFilter =xpcfComponentManager->resolve<solver::map::IMapFilter>();
#ifdef SOLAR_USE_OPENGL
   auto viewer3DPoints =xpcfComponentManager->resolve<display::I3DPointsViewer>();
#endif

    // declarations of data structures used to exange information between components
    SRef<Image>                                  image1;
    SRef<Image>                                  image2;

    std::vector<Keypoint>                        keypoints1;
    std::vector<Keypoint>                        keypoints2;

    SRef<DescriptorBuffer>                       descriptors1;
    SRef<DescriptorBuffer>                       descriptors2;
    std::vector<DescriptorMatch>                 matches;
    std::vector<CloudPoint>                      cloud, filteredCloud;

    SRef<Image>                                  matchesImage;

    Transform3Df                                 poseFrame1 = Transform3Df::Identity();
    Transform3Df                                 poseFrame2;

    // initialize components requiring the camera intrinsic parameters (please refeer to the use of intrinsic parameters file)
    poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    triangulator->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    LOG_INFO("SetCameraParameters OK - Images OK");

    // Get first image
    if (imageLoader1->getImage(image1) != FrameworkReturnCode::_SUCCESS)
    {
        LOG_ERROR("Cannot load image 1 with path {}", imageLoader1->bindTo<xpcf::IConfigurable>()->getProperty("pathFile")->getStringValue());
        return -1;
    }

    // Get second image
    if (imageLoader2->getImage(image2) != FrameworkReturnCode::_SUCCESS)
    {
        LOG_ERROR("Cannot load image 2 with path {}", imageLoader2->bindTo<xpcf::IConfigurable>()->getProperty("pathFile")->getStringValue());
        return -1;
    }
    // Detect the keypoints for the first image
    keypointsDetector->detect(image1, keypoints1);
    // Detect the keypoints for the second image
    keypointsDetector->detect(image2, keypoints2);
    // Compute the descriptor for each keypoint extracted from the first image
    descriptorExtractor->extract(image1, keypoints1, descriptors1);
    // Compute the descriptor for each keypoint extracted from the second image
    descriptorExtractor->extract(image2, keypoints2, descriptors2);
    // Compute the matches between the keypoints of the first image and the keypoints of the second image
    matcher->match(descriptors1, descriptors2, matches);
    int nbMatches = (int)matches.size();
    // Estimate the pose of the second frame (the first frame being the reference of our coordinate system)
    poseFinderFrom2D2D->estimate(keypoints1, keypoints2, poseFrame1, poseFrame2, matches);
    LOG_INFO("Number of matches used for triangulation {}//{}", matches.size(), nbMatches);
    LOG_INFO("Estimated pose of the camera for the frame 2: \n {}", poseFrame2.matrix());
    // Create a image showing the matches used for pose estimation of the second camera
    overlayMatches->draw(image1, image2, matchesImage, keypoints1, keypoints2, matches);

    // Triangulate the inliers keypoints which match
    double reproj_error = triangulator->triangulate(keypoints1,keypoints2,matches,std::make_pair(0, 1),poseFrame1,poseFrame2,cloud);
    LOG_INFO("Reprojection error: {}", reproj_error);
    mapFilter->filter(poseFrame1, poseFrame2, cloud, filteredCloud);

    // Display the matches and the 3D point cloud
    while (true){
        if (
#ifdef SOLAR_USE_OPENGL
            viewer3DPoints->display(filteredCloud, poseFrame2) == FrameworkReturnCode::_STOP ||
#endif
            viewerMatches->display(matchesImage) == FrameworkReturnCode::_STOP  )
        {
           LOG_INFO("End of Triangulation sample");
           break;
        }
    }
    return 0;
}



