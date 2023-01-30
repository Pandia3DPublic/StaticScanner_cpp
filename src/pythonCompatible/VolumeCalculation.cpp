#include "VolumeCalculation.h"
#include "GlobalDefines.h"
#include "util.h"
#include "GUI/guiUtil.h"
#include "../volume/frustum.h"

using namespace std;

double getCurrentVolume(CameraWrapper &cam)
{
    using namespace std::chrono_literals;
    //for both pcds build z-buffers. Keep a pixel only if the recorded value is some minimum value smaller
    //than the reference value.
    Eigen::Matrix3d &intr = cam.DepthLDTIntrinsic.intrinsic_matrix_;
    auto combinedPcd = make_shared<open3d::geometry::PointCloud>();
    Eigen::Matrix4d ReftoCam = readMatrixFromDisc(ResourceFolderPath + "config/SiloAlignmentFine.txt");
    string modelName = readStringFromDisc(ResourceFolderPath + "config/", "modelname.txt");
    if(modelName.size() == 0){
        cout << "Aborting since silo model name is not set! \n";
        return 0;
    }
    Eigen::MatrixXd referenceDepth = getFOVDepthBuffer(cam, *getReferenceMesh(modelName), ReftoCam.inverse());

    open3d::geometry::Image recordedDepth = getAverageImg(cam.rgbdImages)->depth_;

    // auto recordedDepth = cam.rgbdImages.front()->depth_;
    auto rcolor = cam.rgbdImages.front()->color_;

    // auto filteredRecordedDepth = make_shared<open3d::geometry::Image>();
    // filteredRecordedDepth->Prepare(recordedDepth.width_, recordedDepth.height_, 1, 4);
    // auto filteredReferenceDepth = make_shared<open3d::geometry::Image>();
    // filteredReferenceDepth->Prepare(recordedDepth.width_, recordedDepth.height_, 1, 4);

    //Possbile improvements:
    //1. adjust for wallnoise volume loses, since data gets cut
    //2. try to filter small extra clusters
    //3. If there are large parts with no data, we neglect those parts in volume calculation, try 2d convex hull (optional)
    double volume = 0;
    cv::Mat vol_img(recordedDepth.height_, recordedDepth.width_, CV_8UC3, cv::Scalar(0));
    // cv::Mat vol_img(recordedDepth.width_, recordedDepth.height_, CV_8UC3, cv::Scalar(0));

    for (int i = 0; i < referenceDepth.rows(); i++)
    {
        for (int j = 0; j < referenceDepth.cols(); j++)
        {

            double d = *recordedDepth.PointerAt<float>(j, i) - referenceDepth(i, j);
            if (d<g_VolumeNoise && * recordedDepth.PointerAt<float>(j, i)> 1e-3 && referenceDepth(i, j) > 1e-3)
            {
                //just for vis
                combinedPcd->points_.push_back(PixeltoPoint(i, j, referenceDepth(i, j), cam.DepthLDTIntrinsic.intrinsic_matrix_));
                combinedPcd->points_.push_back(PixeltoPoint(i, j, *recordedDepth.PointerAt<float>(j, i), cam.DepthLDTIntrinsic.intrinsic_matrix_));
                combinedPcd->colors_.push_back(Eigen::Vector3d(0.5, 0.5, 0.5));
                Eigen::Vector3d c((double)*rcolor.PointerAt<uchar>(j, i, 0), (double)*rcolor.PointerAt<uchar>(j, i, 1), (double)*rcolor.PointerAt<uchar>(j, i, 2));
                c = c / 255.0;
                combinedPcd->colors_.push_back(c);
                vol_img.at<cv::Vec3b>(i,j) = cv::Vec3b(153, 255, 102); 
                //the actual images
                // *filteredRecordedDepth->PointerAt<float>(j, i) = *recordedDepth.PointerAt<float>(j, i);
                // *filteredReferenceDepth->PointerAt<float>(j, i) = referenceDepth(i, j);

                //construct frustum per image, near plane is recorded, far plane is reference
                //i is for height, j is for width
                Frustum f(std::abs(d), *recordedDepth.PointerAt<float>(j, i) / intr(1, 1),
                          *recordedDepth.PointerAt<float>(j, i) / intr(0, 0),
                          referenceDepth(i, j) / intr(1, 1),
                          referenceDepth(i, j) / intr(0, 0));
                volume += f.getVolume();
            }
        }
    }

    cv::Mat vol_color_img;
    if(cam.CameraType == EnsensoCamera){
        vol_color_img = cam.FRCalibrationImages.front();
    }else{
        vol_color_img = Open3DToOpenCV(cam.rgbdImages.front()->color_);
    }
    float alpha = 0.5;
    float beta = 1-alpha;


    cv::resize(vol_img, vol_img, vol_color_img.size());
    for (int i = 0; i < vol_img.rows; i++)
    {
        for (int j = 0; j < vol_img.cols; j++)
        {
            if(vol_img.at<cv::Vec3b>(i,j)(0) != 0){
                vol_img.at<cv::Vec3b>(i, j) = alpha * vol_img.at<cv::Vec3b>(i, j) + beta * vol_color_img.at<cv::Vec3b>(i, j);
            } else{
                vol_img.at<cv::Vec3b>(i, j) = vol_color_img.at<cv::Vec3b>(i, j);
            }
        }
    }
    cam.VolumeImage = vol_img;
    return volume;
    // auto refpcd = open3d::geometry::PointCloud::CreateFromDepthImage(*filteredReferenceDepth, cam.DepthLDTIntrinsic);
    // auto recpcd = open3d::geometry::PointCloud::CreateFromDepthImage(*filteredRecordedDepth, cam.DepthLDTIntrinsic);
    // open3d::visualization::DrawGeometries({refpcd,recpcd,getOrigin()});
}

shared_ptr<open3d::geometry::TriangleMesh> getReferenceMesh(string name)
{
    string modelfolder("/../resources/models/");
    auto o3dmesh = readMesh(get_current_dir_name() + modelfolder + name);
    o3dmesh->Translate(-o3dmesh->GetCenter());
    o3dmesh->Scale(0.001, o3dmesh->GetCenter());
    o3dmesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    o3dmesh->ComputeTriangleNormals();
    o3dmesh->NormalizeNormals();
    return o3dmesh;
}

Eigen::MatrixXd getFOVDepthBuffer(CameraWrapper &cam, open3d::geometry::TriangleMesh mesh, const Eigen::Matrix4d &refToCamTrans)
{

    Eigen::MatrixXd depth_buffer = Eigen::MatrixXd::Constant(cam.DepthLDTIntrinsic.height_, cam.DepthLDTIntrinsic.width_, 0);
    mesh.Transform(refToCamTrans);

    for (int i = 0; i < mesh.triangles_.size(); i++)
    {

        Eigen::Vector3d v03D = mesh.vertices_[mesh.triangles_[i](0)];
        Eigen::Vector3d v13D = mesh.vertices_[mesh.triangles_[i](1)];
        Eigen::Vector3d v23D = mesh.vertices_[mesh.triangles_[i](2)];

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
