#include "ScanningSingleton.h"
#include "util.h"
#include "calibration/calibutil.h"
// #include <teaser/registration.h>
#include <Eigen/Eigenvalues>

using namespace std;
ScanningSingleton::ScanningSingleton(std::shared_ptr<std::vector<CameraWrapper>> cameras_)
{
    CurrentVoxelGrid = make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(voxel_length, d_trunc,
                                                                                       open3d::pipelines::integration::TSDFVolumeColorType::RGB8, 8, 1); //todo test different sampling strides in 4-th argument.

    CurrentPCD = make_shared<open3d::geometry::PointCloud>();
    CurrentDownsampledPCD = make_shared<open3d::geometry::PointCloud>();
    OldPCD = make_shared<open3d::geometry::PointCloud>();
    OldDownsampledPCD = make_shared<open3d::geometry::PointCloud>();
    ReferencePCD = make_shared<open3d::geometry::PointCloud>();
    SmallReferencePCD = make_shared<open3d::geometry::PointCloud>();
    ReferenceMesh = make_shared<open3d::geometry::TriangleMesh>();

    Cameras = cameras_;
}

ScanningSingleton::~ScanningSingleton()
{
}

void ScanningSingleton::reset()
{
    CurrentPCD->Clear();
    CurrentDownsampledPCD->Clear();
    OldPCD->Clear();
    OldDownsampledPCD->Clear();
    CurrentVoxelGrid->Reset();
}

void ScanningSingleton::ComputeDistanceToReference()
{
    // std::vector<Eigen::Vector3d> nearestPoints;
    // distances = getDistanceFromMesh(ReferenceMesh, CurrentPCD, nearestPoints);

    // distances = CurrentPCD->ComputePointCloudDistance(*ReferencePCD);
    distances = getPointCloudPointToPlaneDistances(CurrentPCD, ReferencePCD); // todo has artifacts
    // distances = getPointCloudPointToPlaneDistancesBidirectional(CurrentPCD, ReferencePCD); //todo test this
}

void ScanningSingleton::ColorCurrentPCDGradient()
{
    float midColorThreshold = (g_ColorMaxThreshold - g_ColorMinThreshold) / 2.0;
    float colordiff = g_ColorMaxThreshold - g_ColorMinThreshold;
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] > g_ColorMaxThreshold)
        {
            CurrentPCD->colors_[i] = Eigen::Vector3d(1, 0, 0); //red
        }
        else if (distances[i] < g_ColorMinThreshold)
        {
            CurrentPCD->colors_[i] = Eigen::Vector3d(0, 1.0, 0); ///green
        }
        else if (distances[i] <= g_ColorMinThreshold + colordiff / 2)
        {
            CurrentPCD->colors_[i] = Eigen::Vector3d(((distances[i] - g_ColorMinThreshold) * 2) / colordiff, 1, 0); //green base, increase red
        }
        else if (distances[i] > g_ColorMinThreshold + colordiff / 2)
        {
            CurrentPCD->colors_[i] = Eigen::Vector3d(1, 1 - (distances[i] - g_ColorMinThreshold - colordiff / 2) / colordiff, 0); //red base, decrease green
        }
    }
}

void ScanningSingleton::ColorCurrentPCDwithDistance()
{
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] > g_ColorMaxThreshold)
        {
            CurrentPCD->colors_[i] = Eigen::Vector3d(1, 0, 0); //red
        }
        else
        {
            CurrentPCD->colors_[i] = Eigen::Vector3d(0, 1, 0); //green
        }
    }
}

bool InsideDepthImageIntr(const Eigen::Vector2d &p,const open3d::camera::PinholeCameraIntrinsic &intr)
{
    if (p(0) > 0 && p(0) < intr.height_ && p(1) > 0 && p(1) < intr.width_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

Eigen::MatrixXd getDepthBufferFromPCD(const open3d::geometry::PointCloud& pcd, const open3d::camera::PinholeCameraIntrinsic & intr)
{
    Eigen::MatrixXd depth_buffer = Eigen::MatrixXd::Constant(intr.height_, intr.width_, 0);

    for (int i = 0; i < pcd.points_.size(); i++)
    { 
        Eigen::Vector2d ind = PointtoPixelExact(pcd.points_[i], intr.intrinsic_matrix_);
        if (InsideDepthImageIntr(ind,intr))
        {
            depth_buffer((int)ind(0), (int)ind(1)) = pcd.points_[i](2);
        }
    }
    return depth_buffer;
}

Eigen::MatrixXd ScanningSingleton::getFOVDepthBuffer(CameraWrapper &cam)
{
    Eigen::Matrix4d refToCamTrans = cam.CameraToReferenceTrans.inverse(); 
    if(ReferenceMesh->IsEmpty()){
        auto transformedRefPcd = *ReferencePCD;
        transformedRefPcd.Transform(refToCamTrans);
        return getDepthBufferFromPCD(transformedRefPcd, cam.DepthLDTIntrinsic);
    }

    Eigen::MatrixXd depth_buffer = Eigen::MatrixXd::Constant(cam.DepthLDTIntrinsic.height_, cam.DepthLDTIntrinsic.width_, 0);

    auto transformedRefMesh = *ReferenceMesh;
    transformedRefMesh.Transform(refToCamTrans);

    for (int i = 0; i < transformedRefMesh.triangles_.size(); i++)
    {

        Eigen::Vector3d v03D = transformedRefMesh.vertices_[transformedRefMesh.triangles_[i](0)];
        Eigen::Vector3d v13D = transformedRefMesh.vertices_[transformedRefMesh.triangles_[i](1)];
        Eigen::Vector3d v23D = transformedRefMesh.vertices_[transformedRefMesh.triangles_[i](2)];

        Eigen::Vector2d v0 = PointtoPixelExact(v03D, cam.DepthLDTIntrinsic.intrinsic_matrix_);
        Eigen::Vector2d v1 = PointtoPixelExact(v13D, cam.DepthLDTIntrinsic.intrinsic_matrix_);
        Eigen::Vector2d v2 = PointtoPixelExact(v23D, cam.DepthLDTIntrinsic.intrinsic_matrix_);

        //now we have the pixel coordinates of the projected vertices
        int xmin = std::min(std::min(v0(0), v1(0)), v2(0));
        int ymin = std::min(std::min(v0(1), v1(1)), v2(1));
        int xmax = std::ceil(std::max(std::max(v0(0), v1(0)), v2(0)));
        int ymax = std::ceil(std::max(std::max(v0(1), v1(1)), v2(1)));
        double area = edgeFunction(v0, v1, v2);
        for (int i = xmin; i < xmax; i++)
        {
            for (int j = ymin; j < ymax; j++)
            {
                Eigen::Vector2d p(i + 0.5, j + 0.5);
                if (cam.InsideDepthImage(p))
                {
                    double w0 = edgeFunction(v1, v2, p) / area;
                    double w1 = edgeFunction(v2, v0, p) / area;
                    double w2 = edgeFunction(v0, v1, p) / area;
                    //check if in triangle
                    if (w0 >= 0 && w1 >= 0 && w2 >= 0 && w0 <= 1 && w1 <= 1 && w2 <= 1)
                    {
                        // check if in image plane
                        double dinv = w0 * 1 / v03D(2) + w1 * 1 / v13D(2) + w2 * 1 / v23D(2);
                        double d = 1 / dinv;
                        if (d < depth_buffer(i, j) || depth_buffer(i, j) == 0)
                        {
                            depth_buffer(i, j) = d;
                        }
                    }
                }
            }
        }
    }

    return depth_buffer;
}
// the following points on the object surface can be marked as missing:
// 1. At least one camera gets data for the point. This can be checked by projecting the point on a
// camera screen and checking if the pixel depht value is non zero.
// 2. The depth value strongly diverges from the should be value.
// Tactic: For each cam generate a reference pcd depth buffer. For each non zero pixel in the
// buffer check if the corresponding scannedrgbd pixel is non zero and diverges above a certain
// threshold. If it does, mark the corresponding point on the reference pcd as missing / add the
// corresponding point to the missing pcd.
shared_ptr<open3d::geometry::PointCloud> ScanningSingleton::getMissingPoints()
{

    shared_ptr<open3d::geometry::PointCloud> missingPoints = make_shared<open3d::geometry::PointCloud>();
    for (auto &cam : *Cameras)
    {
        Eigen::MatrixXd depth_buffer = getFOVDepthBuffer(cam);
        // open3d::visualization::DrawGeometries({EigenToO3DDepthImage(depth_buffer)});
        // open3d::visualization::DrawGeometries({cam.ScannedRGBD});
        // open3d::visualization::DrawGeometries({EigenDepthToPCD(depth_buffer,cam.DepthLDTIntrinsic),cam.ScannedRGBD });

        auto rgbd = cam.rgbdImages.front();

        for (int i = 0; i < depth_buffer.rows(); i++)
        {
            for (int j = 0; j < depth_buffer.cols(); j++)
            {
                double &d = depth_buffer(i, j);
                if (d != 0 && *cam.ScannedRGBD->depth_.PointerAt<float>(j, i) != 0)
                {
                    if (std::abs(d - *cam.ScannedRGBD->depth_.PointerAt<float>(j, i)) > g_ColorMaxThreshold)
                    {
                                Eigen::Vector3d p = PixeltoPoint(i, j, d, cam.DepthLDTIntrinsic.intrinsic_matrix_);
                        p = cam.Extrinsic.block<3, 3>(0, 0) * p + cam.Extrinsic.block<3, 1>(0, 3);
                        missingPoints->points_.push_back(p);
                    }
                }
            }
        }
    }

    return missingPoints;
}

//note that at high angles we get missing point coloration due to depth image resolution (2 points one bucket)
void ScanningSingleton::ColorFOV()
{
    for (auto &cam : *Cameras)
    {
        Eigen::MatrixXd depth_buffer = getFOVDepthBuffer(cam);
        auto depth_image = EigenToO3DDepthImage(depth_buffer);
        applyNormalFilter(*depth_image, cam.DepthLDTIntrinsic);
        depth_buffer = O3DDepthtoEigen(depth_image);
        // open3d::visualization::DrawGeometries({depth_image});
        // auto d2 = EigenToO3DDepthImage(depth_buffer);
        // open3d::visualization::DrawGeometries({d2});

        Eigen::Matrix4d refToCamTrans = cam.CameraToReferenceTrans.inverse();
        // open3d::visualization::DrawGeometries({EigenDepthToPCD(depth_buffer,cam.DepthLDTIntrinsic), getOrigin()});

        double epsilon = 1e-3; // in case point lies exactly on mesh
        for (int i = 0; i < ReferencePCD->points_.size(); i++)
        {
            Eigen::Vector3d pt = refToCamTrans.block<3, 3>(0, 0) * ReferencePCD->points_[i] + refToCamTrans.block<3, 1>(0, 3);
            Eigen::Vector2d ind = PointtoPixelExact(pt, cam.DepthLDTIntrinsic.intrinsic_matrix_);
            if (cam.InsideDepthImage(ind))
            {
                if (pt(2) <= depth_buffer((int)ind(0), (int)ind(1)) + epsilon)
                {
                    ReferencePCD->colors_[i] = Eigen::Vector3d(0, 1, 0); //green
                }
            }
        }
    }
}

void ScanningSingleton::RecolorCurrentPCD()
{
    CurrentPCD->colors_ = CurrentPCDColors;
}

//todo eigen vector stuff
vector<Eigen::Vector2i> getEdgePixels(shared_ptr<open3d::geometry::RGBDImage> img)
{

    auto &depth = img->depth_;
    vector<Eigen::Vector2i> out;
    float thres = 1e-3;

    for (int i = 1; i < depth.height_ - 1; i++)
    {
        for (int j = 1; j < depth.width_ - 1; j++)
        {
            int count = 0;
            if (*depth.PointerAt<float>(j, i) != 0)
            {
                if (*depth.PointerAt<float>(j + 1, i) < thres)
                    count++;
                if (*depth.PointerAt<float>(j, i + 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j + 1, i + 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j - 1, i) < thres)
                    count++;
                if (*depth.PointerAt<float>(j, i - 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j - 1, i - 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j + 1, i - 1) < thres)
                    count++;
                if (*depth.PointerAt<float>(j - 1, i + 1) < thres)
                    count++;
            }
            if (count >= 1)
                out.push_back(Eigen::Vector2i(j, i));
        }
    }
    return out;
}

vector<Eigen::Vector2i> getBackgroundEdgePixels(shared_ptr<open3d::geometry::RGBDImage> img, vector<Eigen::Vector2i> &edges)
{
    auto &depth = img->depth_;
    auto cvDepth = Open3DToOpenCV(img->depth_).clone();
    vector<int> innerToOuterEdge;
    //1. Make binary image where only the edge is non zero
    cv::Mat binary(cvDepth.size(), CV_8U);
    binary = 255; //all filled except the edge
    for (auto &edge : edges)
    {
        binary.at<u_int8_t>(edge(1), edge(0)) = 0;
    }
    cv::Mat dist;

    cv::distanceTransform(binary, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::Mat ring;
    cv::inRange(dist, 9.5, 10.5, ring);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(ring, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    vector<Eigen::Vector2i> out;
    for (int i = 0; i < contours[0].size(); i++)
    {
        out.push_back(Eigen::Vector2i(contours[0][i].x, contours[0][i].y));
    }

    return out;
}

void applyEdgeColorPersistenceFilter(shared_ptr<open3d::geometry::RGBDImage> rgbd)
{

    float edgeSpacing = 10;
    float colorThreshold = 150;
    int n_iterations = 6;
    auto edges = getEdgePixels(rgbd);
    auto bgEdges = getBackgroundEdgePixels(rgbd, edges);
    auto edgeMap = getInnerToOuterEdgeMapping(edges, bgEdges);

    int counter = 1e6;
    int firstIterationCounter = 0;
    int iteration = 0;
    while (counter > firstIterationCounter / 10 && iteration < n_iterations)
    {
        counter = 0;
        for (int i = 0; i < edges.size(); i++)
        {
            float r = *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 0);
            float g = *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 1);
            float b = *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 2);

            float r2 = *rgbd->color_.PointerAt<u_int8_t>(bgEdges[edgeMap[i]](0), bgEdges[edgeMap[i]](1), 0);
            float g2 = *rgbd->color_.PointerAt<u_int8_t>(bgEdges[edgeMap[i]](0), bgEdges[edgeMap[i]](1), 1);
            float b2 = *rgbd->color_.PointerAt<u_int8_t>(bgEdges[edgeMap[i]](0), bgEdges[edgeMap[i]](1), 2);

            double d = std::sqrt((r - r2) * (r - r2) + (g - g2) * (g - g2) + (b - b2) * (b - b2));
            if (d < colorThreshold)
            {
                // *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 2) = 255;
                // *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 0) = 0;
                // *rgbd->color_.PointerAt<u_int8_t>(edges[i](0), edges[i](1), 1) = 0;
                *rgbd->depth_.PointerAt<float>(edges[i](0), edges[i](1)) = 0;
                counter++;
            }
        }
        // cout << "Removed " << counter << " pixel with persistence filter \n";
        edges = getEdgePixels(rgbd);
        edgeMap = getInnerToOuterEdgeMapping(edges, bgEdges);
        if (iteration == 0)
            firstIterationCounter = counter;
        iteration++;
    }
}

//the columns of the matrix are the pca vectors
Eigen::Matrix3d getPCAVectors(shared_ptr<open3d::geometry::PointCloud> pcd)
{
    Eigen::MatrixXd pointMatrix(pcd->points_.size(), 3);
    for (int i = 0; i < pcd->points_.size(); i++)
    {
        pointMatrix.block<1, 3>(i, 0) = pcd->points_[i];
    }
    //################# center the pcds #######################
    pointMatrix = pointMatrix.rowwise() - pointMatrix.colwise().mean();
    //#################### perform actual pca ###################
    Eigen::MatrixXd cov = pointMatrix.adjoint() * pointMatrix;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);

    Eigen::Matrix3d pcaVectors= solver.eigenvectors();
    //check if pca vector are right handed, otherwise for them too
    Eigen::Vector3d zCross = pcaVectors.block<3, 1>(0, 0).cross(pcaVectors.block<3, 1>(0, 1));
    if (zCross.dot(pcaVectors.block<3, 1>(0, 2)) < 0)
    {
        cout << "switching source axis handedness \n";
        pcaVectors.block<3, 1>(0, 2) = -pcaVectors.block<3, 1>(0, 2);
    }
    return pcaVectors;
}

//the rotation returned by this is valid for pcds that are transformed to their center zero point
Eigen::Matrix4d getPCABasedRotation(shared_ptr<open3d::geometry::PointCloud> source, shared_ptr<open3d::geometry::PointCloud> target)
{
    Eigen::Matrix3d sourcePCAVectors = getPCAVectors(source);
    Eigen::Matrix3d targetPCAVectors = getPCAVectors(target);

    Eigen::Matrix3d R0 = sourcePCAVectors.inverse();
    Eigen::Matrix3d R = targetPCAVectors * R0;
    Eigen::Matrix4d R4 = Eigen::Matrix4d::Identity();
    R4.block<3, 3>(0, 0) = R;
    return R4;
}

Eigen::Matrix4d getIterativePCABasedAlignment(shared_ptr<open3d::geometry::PointCloud> source, shared_ptr<open3d::geometry::PointCloud> target)
{
    //usually target is refernce model and source is current scan
    Eigen::Matrix3d targetPCAVectors= getPCAVectors(target);

    //find center
    Eigen::Vector3d centerSource = source->GetCenter();
    Eigen::Vector3d centerTarget = target->GetCenter();
    //translate both pcds to center
    auto source_c = std::make_shared<open3d::geometry::PointCloud>(*source);
    auto target_c = std::make_shared<open3d::geometry::PointCloud>(*target);
    source_c->Translate(-centerSource);
    target_c->Translate(-centerTarget);

    // 1. Algin source to target using the initial pca rotation
    Eigen::Matrix4d Rpca = getPCABasedRotation(source_c, target_c);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    int max_it = 12;
    double angelInc = 2*PANDIA_PI / (max_it - 1);
    double fitness = 0;
    double thres_fitness = 0.95;
    double thres_icp_inlier = 0.02;

    double bestFitness = 0;
    Eigen::Matrix4d bestT = Eigen::Matrix4d::Identity();

    //axis1 is a smaller axis. Just do a flip around that axis.
    for (int axis1 = 0; axis1 < 2; axis1++)
    {
        Eigen::Matrix4d Raxis1 = getRotationMatrixFromVectorAndAngle(targetPCAVectors.block<3, 1>(0, 1), axis1 * PANDIA_PI);
        for (int axis2 = 0; axis2 < max_it; axis2++) //biggest axis
        {

            Eigen::Matrix4d Raxis2 = getRotationMatrixFromVectorAndAngle(targetPCAVectors.block<3, 1>(0, 2), axis2 * angelInc);
            T = refineRegistrationICPCenteredPCDs(source_c, target_c, fitness, Raxis2 * Raxis1 * Rpca, 30, thres_icp_inlier);
            // cout << fitness << endl;
            // auto sourceTrans = make_shared<open3d::geometry::PointCloud>(*source_c);
            // sourceTrans->Transform(T);
            // sourceTrans->PaintUniformColor(Eigen::Vector3d(1,0,0));
            // open3d::visualization::DrawGeometries({getOrigin(),sourceTrans,target_c});
            
            if (fitness > thres_fitness)
            {
                return getTrans(centerTarget) * T * getTrans(-centerSource);
            } 
            if(fitness > bestFitness)
            {
                bestT = T;
                bestFitness = fitness;
            }
        }
    }

    //return best matrix here in case non was really good
    cout << "Did not find a great alignment candidate in pca align, using best candidate\n";
    return getTrans(centerTarget) * bestT * getTrans(-centerSource);
}

//todo does not work right now! need to get rid of reference extrinsic
void ScanningSingleton::FilterCurrentPcdUsingMulitCamVisibility()
{
    cout << CurrentPCD->points_.size() << endl;
    //compute current depth buffers
    vector<Eigen::MatrixXd> depth_buffers;
    vector<shared_ptr<open3d::geometry::KDTreeFlann>> kdtrees;
    for (int j = 0; j < Cameras->size(); j++)
    {
        depth_buffers.push_back(getFOVDepthBuffer(Cameras->at(j)));
        kdtrees.push_back(make_shared<open3d::geometry::KDTreeFlann>(*Cameras->at(j).ScannedPcd));
        //todo normal filter
    }
    auto newpoints = make_shared<open3d::geometry::PointCloud>();
    int j2count = 0;
    int zerocount = 0;
    for (int i = 0; i < CurrentPCD->points_.size(); i++)
    {
        //for each point check by which cameras it is visible
        int camviscount = 0;
        //check how many of these cameras have a pixel in the vincinity (1cm)
        int camscancount = 0;
        for (int j = 0; j < Cameras->size(); j++)
        {
            Eigen::Matrix4d refToCamTrans = Cameras->at(j).CameraToReferenceTrans.inverse();
            Eigen::Vector3d pt = refToCamTrans.block<3, 3>(0, 0) * CurrentPCD->points_[i] +
                                 refToCamTrans.block<3, 1>(0, 3);

            if (Cameras->at(j).IsVisible(pt, depth_buffers[j]))
            {
                camviscount++;

                std::vector<int> inds;
                std::vector<double> dists;
                int n = kdtrees[j]->SearchKNN(pt, 1, inds, dists);
                if (dists.front() * 1000 < g_ColorMaxThreshold)
                {
                    camscancount++;
                }
            }
        }
        if (camviscount == 0) {
            // cout << "camviscount is zero!" << endl;
            zerocount++;
        }
        //todo why camviscouont = 0??????
        if ((camviscount == camscancount || camscancount >= 2) && camviscount > 0)
        {
            newpoints->points_.push_back(CurrentPCD->points_[i]);
            newpoints->colors_.push_back(CurrentPCD->colors_[i]);
            newpoints->normals_.push_back(CurrentPCD->normals_[i]);
        }
    }
    CurrentPCDColors = newpoints->colors_;
    CurrentPCD = newpoints;
    cout << "zerocount " << zerocount << endl;
}
