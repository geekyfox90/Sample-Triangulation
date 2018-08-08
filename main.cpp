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

#include <iostream>
#include <string>
#include <vector>

#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"
#include "SolARModuleOpengl_traits.h"

#include "xpcf/xpcf.h"

#include "api/image/IImageLoader.h"
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/display/ISideBySideOverlay.h"
#include "api/features/IMatchesFilter.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/I2Dto3DTransformDecomposer.h"
#include "api/solver/map/ITriangulator.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::OPENGL;
using namespace SolAR::MODULES::TOOLS;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv){

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_Triangulation.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_FiducialMarker.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");

 // component creation

    auto camera =xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto imageLoader1 =xpcfComponentManager->create<SolARImageLoaderOpencv>("image1")->bindTo<image::IImageLoader>();
    auto imageLoader2 =xpcfComponentManager->create<SolARImageLoaderOpencv>("image2")->bindTo<image::IImageLoader>();
    auto keypointsDetector =xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto descriptorExtractor =xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
    auto matcher =xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto overlay =xpcfComponentManager->create<SolARSideBySideOverlayOpencv>()->bindTo<display::ISideBySideOverlay>();
    auto viewerOriginalMatches =xpcfComponentManager->create<SolARImageViewerOpencv>("originalMatches")->bindTo<display::IImageViewer>();
    auto viewerRedanduncyFilterMatches =xpcfComponentManager->create<SolARImageViewerOpencv>("redanduncyFilterMatches")->bindTo<display::IImageViewer>();
    auto viewerEpipolarFilterMatches =xpcfComponentManager->create<SolARImageViewerOpencv>("epipolarFilterMatches")->bindTo<display::IImageViewer>();
    auto matchesFilterBasic =xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
    auto matchesFilterGeometric =xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
    auto keypointsReindexer = xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
    auto fundamentalFinder =xpcfComponentManager->create<SolARFundamentalMatrixEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
    auto fundamentalDecomposer =xpcfComponentManager->create<SolARSVDFundamentalMatrixDecomposerOpencv>()->bindTo<solver::pose::I2DTO3DTransformDecomposer>();
    auto mapper =xpcfComponentManager->create<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
    auto viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    // declarations

       SRef<Image>                                         image1;
       SRef<Image>                                         image2;

       std::vector< SRef<Keypoint>>                        keypoints1;
       std::vector< SRef<Keypoint>>                        keypoints2;

       SRef<DescriptorBuffer>                              descriptors1;
       SRef<DescriptorBuffer>                              descriptors2;
       std::vector<DescriptorMatch>                        matches;

       std::vector<SRef<Point2Df>>                         matchedKeypoints1;
       std::vector<SRef<Point2Df>>                         matchedKeypoints2;

       std::vector<SRef<Point2Df>>                         gmatchedKeypoints1;
       std::vector<SRef<Point2Df>>                         gmatchedKeypoints2;

       std::vector<SRef<Point2Df>>                         ggmatchedKeypoints1;
       std::vector<SRef<Point2Df>>                         ggmatchedKeypoints2;
       std::vector<SRef<CloudPoint>>                       gcloud;

       SRef<Image>                                         viewerImage1;
       SRef<Image>                                         viewerImage2;
       SRef<Image>                                         viewerImage3;

       std::vector<DescriptorMatch>                        gmatches;
       std::vector<DescriptorMatch>                        ggmatches;

       CamCalibration                                      K;
       CamDistortion                                       dist;
       Transform2Df                                        F;
       std::vector<Transform3Df>                           poses;

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

    // Detect the keypoints of the first image
    keypointsDetector->detect(image1, keypoints1);

    // Detect the keypoints of the second image
    keypointsDetector->detect(image2, keypoints2);

    // Compute the  descriptor for each keypoint extracted from the first image
    descriptorExtractor->extract(image1, keypoints1, descriptors1);

    // Compute the  descriptor for each keypoint extracted from the second image
    descriptorExtractor->extract(image2, keypoints2, descriptors2);

    // Compute the matches between the keypoints of the first image and the keypoints of the second image
    matcher->match(descriptors1, descriptors2, matches);
    LOG_DEBUG("->original matches: {}",matches.size());

    keypointsReindexer->reindex(keypoints1, keypoints2, matches, matchedKeypoints1, matchedKeypoints2);
    overlay->drawMatchesLines(image1, image2, viewerImage1, matchedKeypoints1, matchedKeypoints2);

    matchesFilterBasic->filter(matches,gmatches,keypoints1, keypoints2);
    LOG_DEBUG("->filtred matches with redanduncy: {}", gmatches.size());

    keypointsReindexer->reindex(keypoints1, keypoints2, gmatches, gmatchedKeypoints1, gmatchedKeypoints2);
    overlay->drawMatchesLines(image1, image2, viewerImage2, gmatchedKeypoints1, gmatchedKeypoints2);

    matchesFilterGeometric->filter(gmatches,ggmatches,keypoints1, keypoints2);
    LOG_DEBUG("filtred matches with epipolar constraint: {}", ggmatches.size());

    keypointsReindexer->reindex(keypoints1, keypoints2, ggmatches, ggmatchedKeypoints1, ggmatchedKeypoints2);
    overlay->drawMatchesLines(image1, image2, viewerImage3, ggmatchedKeypoints1, ggmatchedKeypoints2);

   //Try to estimate fundamental matrix;
    fundamentalFinder->find(ggmatchedKeypoints1, ggmatchedKeypoints2,F);
    LOG_DEBUG("->F:\n {}", F.matrix());
 
	K = camera->getIntrinsicsParameters();
	dist = camera->getDistorsionParameters();

    fundamentalDecomposer->decompose(F,K,dist,poses);
    LOG_DEBUG("Poses size: {}", poses.size());
    for(int k = 0; k <poses.size(); ++k)
        LOG_DEBUG("--pose {} :\n {}", k, poses[k].matrix());

    // Triangulate
    LOG_INFO("Start Triangulation");
    Transform3Df pose_canonique = Transform3Df::Identity();
    for( int k = 0; k < poses.size(); ++k){
        std::pair<int, int> working_view = std::make_pair(0, 1);
        mapper->triangulate(ggmatchedKeypoints1,ggmatchedKeypoints2,ggmatches,working_view,pose_canonique,poses[k],K,dist,gcloud);
    }

    bool process = true;
    while (process){
        if (
            viewer3DPoints->display(gcloud, pose_canonique) == FrameworkReturnCode::_STOP ||
            viewerOriginalMatches->display(viewerImage1) == FrameworkReturnCode::_STOP ||
            viewerRedanduncyFilterMatches->display(viewerImage2) == FrameworkReturnCode::_STOP ||
            viewerEpipolarFilterMatches->display(viewerImage3) == FrameworkReturnCode::_STOP )
        {
           process = false;
           LOG_INFO("End of Triangulation sample");
        }
    }
    return 0;
}



