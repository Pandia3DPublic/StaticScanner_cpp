#include "util.h"
#include "open3d/t/geometry/RaycastingScene.h"
#include "PhoxiUtil.h"
#include <iomanip>
using namespace std;
using namespace cv;

//########## Data Cam ######################
std::string getPicNumberString(int a)
{
    std::string out = std::to_string(a);
    while (out.length() < 6)
    {
        out = "0" + out;
    }
    return out;
}

void setFromIntrinsicFile(const string &filepath, open3d::camera::PinholeCameraIntrinsic &intrinsic)
{
    string stringline;
    ifstream cameraIntrinsic(filepath);
    int linecounter = 0;
    if (cameraIntrinsic.is_open())
    {
        while (getline(cameraIntrinsic, stringline))
        {
            IntrinsicFileHelperFunction(stringline, linecounter, intrinsic);
            linecounter++;
        }
    }
    cameraIntrinsic.close();
}

//assigns intrinsic-data from .txt-File
void IntrinsicFileHelperFunction(string value, int lineCounter, open3d::camera::PinholeCameraIntrinsic &intrinsic)
{
    int row = lineCounter - 2;
    if (lineCounter == 0)
        intrinsic.width_ = stoi(value);
    if (lineCounter == 1)
        intrinsic.height_ = stoi(value);
    if (lineCounter > 1)
    {
        istringstream iss(value);
        for (int column = 0; column < 3; column++)
        {
            double matrixValue;
            iss >> matrixValue;
            intrinsic.intrinsic_matrix_(row, column) = matrixValue;
        }
    }
}

//checks if file exists
bool fileExists(const string &filename)
{
    ifstream file;
    file.open(filename);
    if (file)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool saveMatrixToDisc(string path, string name, const Eigen::Matrix4d &matrix)
{
    cout << "Saving matrix to " << path << "\n";
    ofstream file(path + name);
    if (file.is_open())
    {
        file << matrix << endl;
    }
    else
    {
        cout << "Writing Matrix to file failed! \n";
        return false;
    }
    file.close();
    return true;
}

bool saveVectorToDisc(std::string path, std::string name, const std::vector<float> vector)
{
    ofstream file(path + name);
    if (file.is_open())
    {
        for (auto &i : vector)
        {
            file << i << endl;
        }
    }
    else
    {
        cout << "Writing vector to file failed! \n";
        return false;
    }
    file.close();
    return true;
}

bool saveStringToDisc(std::string path, std::string name, const string &content){

    ofstream file(path + name);
    if (file.is_open())
    {
        file << content << endl;
    }
    else
    {
        cout << "Writing string to file failed! \n";
        return false;
    }
    file.close();
    return true;
}

Eigen::Matrix4d readMatrixFromDisc(const std::string &PathAndName)
{
    ifstream matrix(PathAndName);
    string line;
    std::string delimiter = " ";
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    if (matrix.is_open())
    {
        int lineNumber = 0;
        while (getline(matrix, line))
        {
            //cout << " line " << line << endl;
            int row = lineNumber % 4;
            std::string token;
            size_t pos;
            int col = 0;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                token = line.substr(0, pos);
                if (token.compare(""))
                { //this stupid shit return wrong if strings are equal
                    //	cout << token;// << delimiter;
                    out(row, col) = stod(token);
                    line.erase(0, pos + delimiter.length());
                    col++;
                }
                else
                {
                    //cout << "token is empty" << token << "after \n";
                    line.erase(0, 1);
                }
            }
            //cout << line <<  endl;

            out(row, col) = stod(line); //last entry
            lineNumber++;
        }
    }
    else
    {
        cout << "Warning: Reading a matrix from file went wrong\n";
        return Eigen::Matrix4d::Identity();
    }
    return out;
}

std::string readStringFromDisc(const std::string& path,const std::string& name){
    ifstream input(path + name);
    string out;
    string line;
    if (input.is_open())
    {
        while (getline(input, line))
        {
            out = out + line;
        }
    }
    else
    {
        cout << "Warning: Reading a string from file went wrong\n";
        return string();
    }
    return out;
}
//############# conversion functions #########################

// no copy, sets data pointer in cv mat
cv::Mat Open3DToOpenCV(const open3d::geometry::Image &img)
{
    if (img.num_of_channels_ == 3 && img.bytes_per_channel_ == 1)
    {
        cv::Mat out(img.height_, img.width_, CV_8UC3, (void *)img.data_.data());
        return out;
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 2)
    {
        cv::Mat out(img.height_, img.width_, CV_16U, (void *)img.data_.data());
        return out;
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 4)
    {
        cv::Mat out(img.height_, img.width_, CV_32F, (void *)img.data_.data());
        return out;
    }
    else
    {
        cout << "warning conversion not implemented for input image properties" << endl;
        cv::Mat out;
        return out;
    }
}

// memcopy to o3d
std::shared_ptr<open3d::geometry::Image> OpenCVToOpen3D(const cv::Mat &img)
{
    auto out = make_shared<open3d::geometry::Image>();
    out->Prepare(img.cols, img.rows, img.channels(), img.elemSize() / img.channels());
    if (img.isContinuous())
    {
        memcpy(out->data_.data(), img.data, img.total() * img.elemSize());
    }
    else
    {
        cout << "warning opencv image is not continious!" << endl;
    }
    return out;
}

cv::Mat KinectImageToOpenCV(const k4a::image &im)
{
    cv::Mat out;
    if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32)
    {
        cv::Mat cv_image_with_alpha = cv::Mat(im.get_height_pixels(), im.get_width_pixels(), CV_8UC4, (void *)im.get_buffer());
        cv::cvtColor(cv_image_with_alpha, out, cv::COLOR_BGRA2RGB);
    }
    else if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16)
    {
        out = cv::Mat(im.get_height_pixels(), im.get_width_pixels(), CV_16U, (void *)im.get_buffer(), cv::Mat::AUTO_STEP).clone();
    }
    else
    {
        cout << "Warning: KinectImageToOpenCV for this image format is not supported" << endl;
    }
    return out;
}

std::shared_ptr<open3d::geometry::Image> KinectImageToOpen3D(const k4a::image &im)
{
    auto out = make_shared<open3d::geometry::Image>();
    if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32)
    {
        out->Prepare(im.get_width_pixels(), im.get_height_pixels(), 3, sizeof(uint8_t));
        const uint8_t *ch = im.get_buffer();
        for (int i = 0; i < im.get_height_pixels(); i++)
        {
            for (int j = 0; j < im.get_width_pixels(); j++)
            {
                *out->PointerAt<uint8_t>(j, i, 2) = *ch++; //b
                *out->PointerAt<uint8_t>(j, i, 1) = *ch++; //g
                *out->PointerAt<uint8_t>(j, i, 0) = *ch++; //r
                ch++;                                      //skip alpha channel
            }
        }
    }
    else if (im.get_format() == k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16)
    {
        out->Prepare(im.get_width_pixels(), im.get_height_pixels(), 1, sizeof(uint16_t));
        const uint16_t *d = (uint16_t *)(void *)im.get_buffer();
        for (int i = 0; i < im.get_height_pixels(); i++)
        {
            for (int j = 0; j < im.get_width_pixels(); j++)
            {
                *out->PointerAt<uint16_t>(j, i) = *d++;
            }
        }
    }
    else
    {
        cout << "Warning: KinectImageToOpen3D for this image format is not supported" << endl;
    }
    return out;
}

k4a::image Open3DToKinectImage(const open3d::geometry::Image &img)
{
    k4a::image out;
    if (img.num_of_channels_ == 3 && img.bytes_per_channel_ == 1)
    {
        out = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, img.width_, img.height_, img.width_ * (int)sizeof(uint32_t));
        uint8_t *dst_data = (uint8_t *)(void *)k4a_image_get_buffer(out.handle());
        memset(dst_data, 0, (size_t)img.width_ * (size_t)img.height_ * sizeof(uint32_t));
        for (int i = 0; i < img.height_; i++)
        {
            for (int j = 0; j < img.width_; j++)
            {
                *dst_data++ = *img.PointerAt<uint8_t>(j, i, 2); //b
                *dst_data++ = *img.PointerAt<uint8_t>(j, i, 1); //g
                *dst_data++ = *img.PointerAt<uint8_t>(j, i, 0); //r
                dst_data++;                                     //skip alpha
            }
        }
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 2)
    {
        out = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, img.width_, img.height_, img.width_ * (int)sizeof(uint16_t));
        uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(out.handle());
        memset(dst_data, 0, (size_t)img.width_ * (size_t)img.height_ * sizeof(uint16_t));
        for (int i = 0; i < img.height_; i++)
        {
            for (int j = 0; j < img.width_; j++)
            {
                *dst_data++ = *img.PointerAt<uint16_t>(j, i);
            }
        }
    }
    else if (img.num_of_channels_ == 1 && img.bytes_per_channel_ == 4)
    {
        out = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, img.width_, img.height_, img.width_ * (int)sizeof(uint16_t));
        uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(out.handle());
        memset(dst_data, 0, (size_t)img.width_ * (size_t)img.height_ * sizeof(uint16_t));
        for (int i = 0; i < img.height_; i++)
        {
            for (int j = 0; j < img.width_; j++)
            {
                *dst_data++ = (uint16_t)(*img.PointerAt<float>(j, i) * 1000);
            }
        }
    }
    else
    {
        cout << "Warning: Open3DToKinectImage for this image format is not supported" << endl;
    }
    return out;
}

std::shared_ptr<open3d::geometry::PointCloud> KinectPCDToOpen3D(const k4a::image &k4apcd, const k4a::image *color)
{
    Mat cvPcd = Mat(Size(k4apcd.get_width_pixels(), k4apcd.get_height_pixels()), CV_16UC3, (void *)k4apcd.get_buffer(), Mat::AUTO_STEP);
    auto out = make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < cvPcd.rows; i++)
    {
        for (int j = 0; j < cvPcd.cols; j++)
        {
            if (cvPcd.at<cv::Vec3s>(i, j)(2) != 0)
            {
                out->points_.push_back(Eigen::Vector3d(cvPcd.at<cv::Vec3s>(i, j)(0),
                                                       cvPcd.at<cv::Vec3s>(i, j)(1),
                                                       cvPcd.at<cv::Vec3s>(i, j)(2)) /
                                       1000.0);
            }
        }
    }

    if (color != nullptr && color->get_width_pixels() == k4apcd.get_width_pixels() && color->get_height_pixels() == k4apcd.get_height_pixels())
    {
        Mat cvColor4 = Mat(Size(color->get_width_pixels(), color->get_height_pixels()), CV_8UC4, (void *)color->get_buffer(), Mat::AUTO_STEP);
        Mat cvColor = Mat(Size(color->get_width_pixels(), color->get_height_pixels()), CV_8UC3);
        cvtColor(cvColor4, cvColor, cv::ColorConversionCodes::COLOR_BGRA2RGB); //remove alpha channel and change ordering
        for (int i = 0; i < cvColor.rows; i++)
        {
            for (int j = 0; j < cvColor.cols; j++)
            {
                if (cvPcd.at<cv::Vec3s>(i, j)(2) != 0)
                {

                    out->colors_.push_back(Eigen::Vector3d(cvColor.at<cv::Vec3b>(i, j)(0) / 255.0,
                                                           cvColor.at<cv::Vec3b>(i, j)(1) / 255.0,
                                                           cvColor.at<cv::Vec3b>(i, j)(2) / 255.0));
                }
            }
        }
    }
    return out;
}

// Convert rs2::frame to cv::Mat
cv::Mat FrametoMat(const rs2::frame &f)
{
    using namespace cv;
    using namespace rs2;
    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();
    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        auto r_bgr = Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
        Mat r_rgb;
        cvtColor(r_bgr, r_rgb, COLOR_BGR2RGB);
        return r_rgb;
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat FrametoMatMeters(const rs2::depth_frame &f)
{
    cv::Mat dm = FrametoMat(f);
    dm.convertTo(dm, CV_64F);
    dm = dm * f.get_units();
    return dm;
}

// Converts depth frame to a matrix of doubles with distances in millimeters
cv::Mat FrametoMatMillimeters(const rs2::depth_frame &f)
{
    cv::Mat dm = FrametoMat(f);
    float factor = f.get_units() * 1000;
    dm.convertTo(dm, CV_64F);
    dm = dm * factor;
    dm.convertTo(dm, CV_16S);
    return dm;
}

double toDegrees(const double &radians)
{
    return radians * 180.0 / PANDIA_PI;
}
double toRadians(const double &degrees)
{
    return degrees * PANDIA_PI / 180.0;
}

//very specific for camera readout conversions
Eigen::Matrix4d arraysToEigenRow(float *rot, float *trans)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) << rot[0], rot[1], rot[2],
        rot[3], rot[4], rot[5],
        rot[6], rot[7], rot[8];
    out.block<3, 1>(0, 3) << trans[0], trans[1], trans[2];
    return out;
}

//very specific for camera readout conversions
Eigen::Matrix4d arraysToEigenColumn(float *rot, float *trans)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) << rot[0], rot[3], rot[6],
        rot[1], rot[4], rot[7],
        rot[2], rot[5], rot[8];
    out.block<3, 1>(0, 3) << trans[0], trans[1], trans[2];
    return out;
}

//############################ vis helper ###########################
std::shared_ptr<open3d::geometry::LineSet> getOrigin()
{
    auto ls = std::make_shared<open3d::geometry::LineSet>();
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    Eigen::Vector3d x(1.0, 0.0, 0.0);
    Eigen::Vector3d y(0.0, 1.0, 0.0);
    Eigen::Vector3d z(0.0, 0.0, 1.0);

    Eigen::Vector3d xcolor(1.0, 0.0, 0.0);
    Eigen::Vector3d ycolor(0.0, 1.0, 0.0);
    Eigen::Vector3d zcolor(0.0, 0.0, 1.0);

    ls->points_.push_back(zero);
    ls->points_.push_back(x);
    ls->points_.push_back(y);
    ls->points_.push_back(z);
    ls->lines_.push_back(Eigen::Vector2i(0, 1));
    ls->lines_.push_back(Eigen::Vector2i(0, 2));
    ls->lines_.push_back(Eigen::Vector2i(0, 3));
    ls->colors_.push_back(xcolor);
    ls->colors_.push_back(ycolor);
    ls->colors_.push_back(zcolor);

    return ls;
}

std::shared_ptr<open3d::geometry::LineSet> getLineVis(const Eigen::Matrix3d &axis)
{

    auto ls = std::make_shared<open3d::geometry::LineSet>();
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    Eigen::Vector3d x = axis.block<3, 1>(0, 0);
    Eigen::Vector3d y = axis.block<3, 1>(0, 1);
    Eigen::Vector3d z = axis.block<3, 1>(0, 2);

    Eigen::Vector3d xcolor(0.0, 0.0, 0.0); //black
    Eigen::Vector3d ycolor(0.0, 1.0, 1.0); //green-blue
    Eigen::Vector3d zcolor(1.0, 0.0, 1.0); //pink

    ls->points_.push_back(zero);
    ls->points_.push_back(x);
    ls->points_.push_back(y);
    ls->points_.push_back(z);
    ls->lines_.push_back(Eigen::Vector2i(0, 1));
    ls->lines_.push_back(Eigen::Vector2i(0, 2));
    ls->lines_.push_back(Eigen::Vector2i(0, 3));
    ls->colors_.push_back(xcolor);
    ls->colors_.push_back(ycolor);
    ls->colors_.push_back(zcolor);

    return ls;
}

shared_ptr<open3d::geometry::TriangleMesh> getCameraArrow(const Eigen::Matrix4d &trans)
{
    auto arrow = open3d::geometry::TriangleMesh::CreateArrow(0.05, 0.075, 0.2, 0.1);
    arrow->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
    arrow->Transform(trans);
    return arrow;
}
//######################### misc ############################
CameraWrapper *getCameraBySerialNumber(std::shared_ptr<std::vector<CameraWrapper>> cameras, std::string SerialNumber)
{
    for (auto &cam : *cameras)
    {
        if (cam.SerialNumber == SerialNumber)
        {
            return &cam;
        }
    }
    return nullptr;
}

bool SortbySerialNumber(const CameraWrapper &c1, const CameraWrapper &c2)
{
    return (c1.SerialNumber < c2.SerialNumber);
}

bool SortbyAlphabet(const std::string &s1, const std::string &s2)
{
    return (s1 < s2);
}

int getNumberofConnectedDevices(CameraTypes camtype)
{
    if (camtype == AzureKinect)
    {
        try
        {
            return k4a_device_get_installed_count();
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in getNumberofConnectedDevices: " << e.what() << '\n';
        }
    }
    if (camtype == IntelCamera)
    {
        try
        {
            return CameraWrapper::IntelContext.query_devices().size();
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in getNumberofConnectedDevices: " << e.what() << '\n';
        }
    }
    if (camtype == DataCamera)
    {
        cout << "Number of cams set to 1 for datacam\n";
        return 1;
    }
    if (camtype == PhoxiScanner)
    {
        try
        {
            //Check if the PhoXi Control Software is running
            if (!CameraWrapper::PhoxiFactory.isPhoXiControlRunning())
            {
                std::cout << "PhoXi Control Software is not running" << std::endl;
                return 0;
            }
            //Get List of available devices on the network
            auto DeviceList = CameraWrapper::PhoxiFactory.GetDeviceList();
            if (DeviceList.empty())
            {
                std::cout << "PhoXi Factory has found 0 devices" << std::endl;
                return 0;
            }
            bool includeFileCams = false;
            if (includeFileCams)
            {
                CameraWrapper::PhoxiDeviceList = DeviceList;
            }
            else
            {
                for (int i = 0; i < DeviceList.size(); i++)
                {
                    if (!DeviceList.at(i).IsFileCamera)
                    {
                        CameraWrapper::PhoxiDeviceList.push_back(DeviceList.at(i));
                    }
                }
            }
            printDeviceInfoList(CameraWrapper::PhoxiDeviceList);
            return CameraWrapper::PhoxiDeviceList.size();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in getNumberofConnectedDevices: " << e.what() << '\n';
        }
    }
    if (camtype == ZividCamera)
    {
        try
        {
            CameraWrapper::ZividCameraList = CameraWrapper::ZividApp.cameras();
            return CameraWrapper::ZividCameraList.size();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in getNumberofConnectedDevices: " << Zivid::toString(e) << std::endl;
        }
    }
    if (camtype == EnsensoCamera)
    {
        try
        {
            cout << "Opening NxLib and waiting for cameras to be detected\n";
            nxLibInitialize(true);
            CameraWrapper::EnsensoRoot = NxLibItem();
            CameraWrapper::EnsensoCameraList = NxLibItem("/Cameras/BySerialNo");
            return CameraWrapper::EnsensoCameraList.count() - 2; // minus two as there are two additional non direct camera nodes attached to /Cameras/
        }
        catch (NxLibException& ex)
        {
            cout << "Error: getNumberofConnectedDevices failed " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        }
    }
    return 0;
}

Eigen::Matrix4d getflip()
{
    Eigen::Matrix4d flip;
    flip << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    return flip;
}

Eigen::Matrix3d getflip3()
{
    Eigen::Matrix3d flip;
    flip << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    return flip;
}
//i is height, j is width. open3d pointer at takes the arguments in reverse
Eigen::Vector3d PixeltoPoint(int &i, int &j, const float &depth, const Eigen::Matrix3d &cameraMatrix)
{
    return Eigen::Vector3d((j - cameraMatrix(0, 2)) * depth / cameraMatrix(0, 0),
                           (i - cameraMatrix(1, 2)) * depth / cameraMatrix(1, 1),
                           depth);
}

//i is height, j is width. open3d pointer_at takes the arguments in reverse
Eigen::Vector2i PointtoPixelRounded(const Eigen::Vector3d &p, const Eigen::Matrix3d &cameraMatrix)
{
    return Eigen::Vector2i(std::round(p(1) / p(2) * cameraMatrix(1, 1) + cameraMatrix(1, 2)),
                           std::round(p(0) / p(2) * cameraMatrix(0, 0) + cameraMatrix(0, 2)));
}

//i is height, j is width. open3d pointer_at takes the arguments in reverse
Eigen::Vector2d PointtoPixelExact(const Eigen::Vector3d &p, const Eigen::Matrix3d &cameraMatrix)
{
    return Eigen::Vector2d(p(1) / p(2) * cameraMatrix(1, 1) + cameraMatrix(1, 2),
                           p(0) / p(2) * cameraMatrix(0, 0) + cameraMatrix(0, 2));
}

//debug function
void addEdges(shared_ptr<open3d::geometry::RGBDImage> img, vector<Eigen::Vector2i> &edges, const Eigen::Vector3d &color, int depthvalue)
{

    for (auto &edge : edges)
    {
        *img->depth_.PointerAt<float>(edge(0), edge(1)) = depthvalue;
        *img->color_.PointerAt<u_int8_t>(edge(0), edge(1), 0) = color(0);
        *img->color_.PointerAt<u_int8_t>(edge(0), edge(1), 1) = color(1);
        *img->color_.PointerAt<u_int8_t>(edge(0), edge(1), 2) = color(2);
    }
}

vector<int> getInnerToOuterEdgeMapping(vector<Eigen::Vector2i> &innerEdge, vector<Eigen::Vector2i> &outerEdge)
{
    vector<int> out;
    for (auto &edge : innerEdge)
    {
        float dmin = 1e6;
        int idx;
        for (int i = 0; i < outerEdge.size(); i++)
        {
            float d = (outerEdge[i] - edge).norm();
            if (d < dmin)
            {
                dmin = d;
                idx = i;
            }
        }
        out.push_back(idx);
    }
    return out;
}
// get the rotation matrix around an arbitrary vector u for an angle a.
Eigen::Matrix4d getRotationMatrixFromVectorAndAngle(const Eigen::Vector3d &v_ex, const double &a)
{
    Eigen::Vector3d k = v_ex;
    k.normalize();
    //2.construct hte cross product matrix
    Eigen::Matrix3d K;
    K << 0, -k(2), k(1),
        k(2), 0, -k(0),
        -k(1), k(0), 0;
    //4. get the rotation matrix using the rodrigues formula
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() + sin(a) * K + (1 - cos(a)) * K * K;
    return out;
}

// get the rotation matrix from a to b by using the Rodrigues rotation formular
//note: This might be the wrong way round and give the transform from b to a
Eigen::Matrix3d getRotationMatrixFromAtoB(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{

    if (a.dot(b) == -a.norm() * b.norm())
    {
        cout << "Warning axis of rotation not defined for parallel vectors \n";
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d out;
    //1.construct axis of rotation
    Eigen::Vector3d k = -a.cross(b);
    k.normalize();
    //2.construct hte cross product matrix
    Eigen::Matrix3d K;
    K << 0, -k(2), k(1),
        k(2), 0, -k(0),
        -k(1), k(0), 0;
    //3.get the angle of roation
    double angle = acos(a.dot(b) / (a.norm() * b.norm()));
    //4. get the rotation matrix using the rodrigues formula
    out = Eigen::Matrix3d::Identity() + sin(angle) * K + (1 - cos(angle)) * K * K;
    return out;
}

// t is ray length parameter
// v0-v2 are vertices
// u and v are baycentric coordinates, which are written to
//taken from the internet without understanding whats going on
bool rayTriangleIntersect(
    const Eigen::Vector3d &orig, const Eigen::Vector3d &dir,
    const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
    float &t, float &u, float &v)
{
    float kEpsilon = 1e-6;
    Eigen::Vector3d v0v1 = v1 - v0;
    Eigen::Vector3d v0v2 = v2 - v0;
    Eigen::Vector3d pvec = dir.cross(v0v2);
    float det = v0v1.dot(pvec);
    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon)
        return false;
    float invDet = 1 / det;

    Eigen::Vector3d tvec = orig - v0;
    u = tvec.dot(pvec) * invDet;
    if (u < 0 || u > 1)
        return false;

    Eigen::Vector3d qvec = tvec.cross(v0v1);
    v = dir.dot(qvec) * invDet;
    if (v < 0 || u + v > 1)
        return false;

    t = v0v2.dot(qvec) * invDet;

    return true;
}

//s = start
//e = end
bool getDistanceToLineSegment(const Eigen::Vector3d &p, const Eigen::Vector3d &s, const Eigen::Vector3d &e, double &dist, Eigen::Vector3d &x)
{
    Eigen::Vector3d f = e - s;
    double fl = f.norm();
    double d = p.dot(f);
    double t = (d - s.dot(f)) / (fl * fl);
    if (t < 0 || t > 1)
        return false;

    // Eigen::Vector3d x = s + t * f;
    x = s + t * f;
    dist = (p - x).norm();
    return true;
}
//todo performance!
vector<double> getDistanceFromMesh(shared_ptr<open3d::geometry::TriangleMesh> mesh, shared_ptr<open3d::geometry::PointCloud> pcd,
                                   std::vector<Eigen::Vector3d> &nearestPoints)
{
    vector<double> out;
    out.resize(pcd->points_.size());
    nearestPoints.clear();
    nearestPoints.resize(pcd->points_.size());
    int maxcount = 0;
//do brute-force triangle ray intersection for all triangles.
#pragma omp parallel for
    for (int i = 0; i < pcd->points_.size(); i++)
    {
        double dmin = 1e6;
        Eigen::Vector3d np = pcd->points_[i]; //nearest point on mesh for debug
        //check for closest distance to all triangles
        for (int j = 0; j < mesh->triangles_.size(); j++)
        {

            Eigen::Vector3d po = pcd->points_[i];
            Eigen::Vector3d n = mesh->triangle_normals_[j];
            Eigen::Vector3d v0 = mesh->vertices_[mesh->triangles_[j](0)];
            Eigen::Vector3d v1 = mesh->vertices_[mesh->triangles_[j](1)];
            Eigen::Vector3d v2 = mesh->vertices_[mesh->triangles_[j](2)];
            float d;
            float u;
            float v;
            bool success = rayTriangleIntersect(po, n, v0, v1, v2, d, u, v);
            if (!success)
            {
                success = rayTriangleIntersect(po, -n, v0, v1, v2, d, u, v);
            }
            if (success && abs(d) < dmin)
            {
                dmin = abs(d);
                np = po + d * n;
            }
        }
        //check for closest distance to all line segments
        //no triangle has been found that is nearest
        for (int j = 0; j < mesh->triangles_.size(); j++)
        {
            for (int k = 0; k < 3; k++)
            {
                double d;
                int index = 0;
                if (k == 0)
                    index = 1;
                if (k == 1)
                    index = 2;
                Eigen::Vector3d s = mesh->vertices_[mesh->triangles_[j](index)];
                Eigen::Vector3d e = mesh->vertices_[mesh->triangles_[j](k)];
                Eigen::Vector3d px;
                bool success = getDistanceToLineSegment(pcd->points_[i], s, e, d, px);
                if (success && abs(d) < dmin)
                {
                    dmin = abs(d);
                    np = px;
                }
            }
        }
        if (dmin < 1e6)
        {
            out[i] = dmin;
            nearestPoints[i] = np;
        }
        else
        {
            //todo need closest point
            maxcount++;
        }
    }
    cout << maxcount << " points did not find the nearest triangle or line segment\n";
    return out;
}

vector<double> getPointCloudPointToPlaneDistances(shared_ptr<open3d::geometry::PointCloud> pcd1, shared_ptr<open3d::geometry::PointCloud> pcd2)
{
    using namespace open3d;
    std::vector<double> distances(pcd1->points_.size());
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*pcd2);
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)pcd1->points_.size(); i++)
    {
        std::vector<int> indices(1);
        std::vector<double> dists(1);
        if (kdtree.SearchKNN(pcd1->points_[i], 1, indices, dists) == 0)
        {
            utility::LogDebug(
                "[ComputePointCloudToPointCloudDistance] Found a point "
                "without neighbors.");
            distances[i] = 0.0;
        }
        else
        {
            Eigen::Vector3d pt = pcd2->points_[indices.front()];
            Eigen::Vector3d nt = pcd2->normals_[indices.front()];
            double rho = pcd1->points_[i].dot(nt);
            distances[i] = std::abs(pt.dot(nt) - rho);
            // distances[i] = std::sqrt(dists[0]);
        }
    }
    return distances;
}

vector<double> getPointCloudPointToPlaneDistancesBidirectional(shared_ptr<open3d::geometry::PointCloud> pcd1, shared_ptr<open3d::geometry::PointCloud> pcd2)
{
    using namespace open3d;
    cout << "Warning: Using untested point to plane distances" << endl;
    // cout << "pcd1 size " << pcd1->points_.size() << endl;
    std::vector<double> distances(pcd1->points_.size(), 1e6);
    std::vector<double> distances1(pcd1->points_.size(), 1e6);
    std::vector<double> distances2(pcd1->points_.size(), 1e6);
    geometry::KDTreeFlann kdtree2;
    kdtree2.SetGeometry(*pcd2);
    int count1 = 0;
    int count2 = 0;
    // open3d::visualization::DrawGeometries({pcd1, getOrigin()});

#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)pcd1->points_.size(); i++)
    {
        std::vector<int> indices2(1);
        std::vector<double> dists2(1);
        if (kdtree2.SearchKNN(pcd1->points_[i], 1, indices2, dists2) == 0)
        {
            utility::LogDebug("[ComputePointCloudToPointCloudDistance] Found a point without neighbors.");
        }
        else
        {
            Eigen::Vector3d pt2 = pcd2->points_[indices2.front()];
            Eigen::Vector3d nt2 = pcd2->normals_[indices2.front()];
            double rho1 = pcd1->points_[i].dot(nt2);
            distances1[i] = std::abs(pt2.dot(nt2) - rho1);

            Eigen::Vector3d pt1 = pcd1->points_[i];
            Eigen::Vector3d nt1 = pcd1->normals_[i];
            double rho2 = pcd2->points_[indices2.front()].dot(nt1);
            distances2[i] = std::abs(pt1.dot(nt1) - rho2);

            if (distances2[i] > 1.5 * distances1[i] || distances1[i] > 1.5 * distances2[i])
            {
                // // point to point dist
                Eigen::Vector3d diff = pcd1->points_[i] - pcd2->points_[indices2.front()];
                distances[i] = diff.norm();
                // count1++;
            }
            else
            {
                distances[i] = std::min(distances1[i], distances2[i]);
                // count2++;
            }
        }
    }
    // cout << "case 1 " << count1 << endl;
    // cout << "case 2 " << count2 << endl;
    return distances;
}

//assumes an untransformed pcd
void applyNormalFilter(shared_ptr<open3d::geometry::PointCloud> &pcd_in)
{
    if (!pcd_in->IsEmpty())
    {
        auto filteredPcd = make_shared<open3d::geometry::PointCloud>();
        for (int j = 0; j < pcd_in->points_.size(); j++)
        {
            if (abs(pcd_in->normals_[j].dot(Eigen::Vector3d(0, 0, 1))) > 0.34)
            { //70deg
                filteredPcd->points_.push_back(pcd_in->points_[j]);
                filteredPcd->colors_.push_back(pcd_in->colors_[j]);
                filteredPcd->normals_.push_back(pcd_in->normals_[j]);
            }
        }
        pcd_in = filteredPcd;
    }
    else
    {
        std::cout << "Warning: PCD is empty in normal filter!" << endl;
    }
}

void applyNormalFilter(shared_ptr<open3d::geometry::RGBDImage> rgbd, open3d::camera::PinholeCameraIntrinsic &intrinsic)
{
    auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, intrinsic);
    if (!pcd->IsEmpty())
    {
        pcd->EstimateNormals();
        pcd->OrientNormalsTowardsCameraLocation();
        applyNormalFilter(rgbd->depth_, pcd, intrinsic);
    }
    else
    {
        std::cout << "Warning: PCD is empty in normal filter!" << endl;
    }
}

void applyNormalFilter(open3d::geometry::Image &depth, open3d::camera::PinholeCameraIntrinsic &intrinsic)
{
    auto pcd = open3d::geometry::PointCloud::CreateFromDepthImage(depth, intrinsic);
    if (!pcd->IsEmpty())
    {
        pcd->EstimateNormals();
        pcd->OrientNormalsTowardsCameraLocation();
        applyNormalFilter(depth, pcd, intrinsic);
    }
    else
    {
        std::cout << "Warning: PCD is empty in normal filter!" << endl;
    }
}

void applyNormalFilter(open3d::geometry::Image &depth, shared_ptr<open3d::geometry::PointCloud> pcd_in, open3d::camera::PinholeCameraIntrinsic &intrinsic)
{
    if (!pcd_in->IsEmpty())
    {
        double thres = cos(toRadians(70));
        for (int i = 0; i < pcd_in->points_.size(); i++)
        {
            if (abs(pcd_in->normals_[i].dot(pcd_in->points_[i])) < thres)
            {
                Eigen::Vector2i idx = PointtoPixelRounded(pcd_in->points_[i], intrinsic.intrinsic_matrix_);
                *depth.PointerAt<float>(idx(1), idx(0)) = 0;
            }
        }
    }
    else
    {
        std::cout << "Warning: PCD is empty in normal filter!" << endl;
    }
}

//get the average of multiple rgbd images
shared_ptr<open3d::geometry::RGBDImage> getAverageImg(vector<shared_ptr<open3d::geometry::RGBDImage>> &rgbds)
{
    if (rgbds.size() == 1)
    {
        return rgbds.front();
    }

    shared_ptr<open3d::geometry::RGBDImage> out = make_shared<open3d::geometry::RGBDImage>();
    out->color_.Prepare(rgbds.front()->color_.width_, rgbds.front()->color_.height_, 3, 1);
    out->depth_.Prepare(rgbds.front()->depth_.width_, rgbds.front()->depth_.height_, 1, 4);

    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            float d_out = 0;
            int count = 0;
            int rgbk = 0;
            for (int k = 0; k < rgbds.size(); k++)
            {
                if (*rgbds[k]->depth_.PointerAt<float>(j, i) != 0)
                {
                    d_out += *rgbds[k]->depth_.PointerAt<float>(j, i);
                    count++;
                    rgbk = k;
                }
            }

            if (count != 0)
            {
                *out->depth_.PointerAt<float>(j, i) = d_out / count;
                *out->color_.PointerAt<u_char>(j, i, 0) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 0);
                *out->color_.PointerAt<u_char>(j, i, 1) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 1);
                *out->color_.PointerAt<u_char>(j, i, 2) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 2);
            }
        }
    }
    return out;
}
//get the average of multiple rgbd images. Depending in the implementation this also filters for inconsistent pixels.
shared_ptr<open3d::geometry::RGBDImage> getSoftAverageImgFilter(vector<shared_ptr<open3d::geometry::RGBDImage>> &rgbds)
{
    if (rgbds.size() == 1)
    {
        return rgbds.front();
    }

    shared_ptr<open3d::geometry::RGBDImage> out = make_shared<open3d::geometry::RGBDImage>();
    out->color_.Prepare(rgbds.front()->color_.width_, rgbds.front()->color_.height_, 3, 1);
    out->depth_.Prepare(rgbds.front()->depth_.width_, rgbds.front()->depth_.height_, 1, 4);

    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            float d_out = 0;
            int count = 0;
            int rgbk = 0;
            for (int k = 0; k < rgbds.size(); k++)
            {
                if (*rgbds[k]->depth_.PointerAt<float>(j, i) != 0)
                {
                    d_out += *rgbds[k]->depth_.PointerAt<float>(j, i);
                    count++;
                    rgbk = k;
                }
            }

            if (count == rgbds.size())
            {
                *out->depth_.PointerAt<float>(j, i) = d_out / count;
                *out->color_.PointerAt<u_char>(j, i, 0) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 0);
                *out->color_.PointerAt<u_char>(j, i, 1) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 1);
                *out->color_.PointerAt<u_char>(j, i, 2) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 2);
            }
        }
    }
    return out;
}
//get the average of multiple rgbd images. Depending in the implementation this also filters for inconsistent pixels.
shared_ptr<open3d::geometry::RGBDImage> getAverageImgFilter(vector<shared_ptr<open3d::geometry::RGBDImage>> &rgbds)
{
    if (rgbds.size() == 1)
    {
        return rgbds.front();
    }

    shared_ptr<open3d::geometry::RGBDImage> out = make_shared<open3d::geometry::RGBDImage>();
    out->color_.Prepare(rgbds.front()->color_.width_, rgbds.front()->color_.height_, 3, 1);
    out->depth_.Prepare(rgbds.front()->depth_.width_, rgbds.front()->depth_.height_, 1, 4);

    for (int i = 0; i < out->depth_.height_; i++)
    {
        for (int j = 0; j < out->depth_.width_; j++)
        {
            float d_out = 0;
            int count = 0;
            int rgbk = 0;
            for (int k = 0; k < rgbds.size(); k++)
            {
                if (*rgbds[k]->depth_.PointerAt<float>(j, i) != 0)
                {
                    d_out += *rgbds[k]->depth_.PointerAt<float>(j, i);
                    count++;
                    rgbk = k;
                }
            }
            bool deviatePixel = false;
            if (count == 0)
            {
                deviatePixel = true;
            }
            else
            {
                d_out /= count;
                for (int k = 0; k < rgbds.size(); k++)
                {
                    if (std::abs(*rgbds[k]->depth_.PointerAt<float>(j, i) - d_out) > 0.015)
                    {
                        deviatePixel = true;
                    }
                }
            }
            if (!deviatePixel)
            {
                // if (count == rgbds.size()) {
                // if(count != 0){
                *out->depth_.PointerAt<float>(j, i) = d_out;
                *out->color_.PointerAt<u_char>(j, i, 0) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 0);
                *out->color_.PointerAt<u_char>(j, i, 1) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 1);
                *out->color_.PointerAt<u_char>(j, i, 2) = *rgbds[rgbk]->color_.PointerAt<u_char>(j, i, 2);
            }
        }
    }
    return out;
}

void showEdges(open3d::geometry::RGBDImage &rgbd)
{
    for (int i = 0; i < rgbd.color_.height_; i++)
    {
        for (int j = 0; j < rgbd.color_.width_; j++)
        {
            if (*rgbd.depth_.PointerAt<float>(j, i) == 0)
            {
                *rgbd.color_.PointerAt<uchar>(j, i, 0) = 0;
                *rgbd.color_.PointerAt<uchar>(j, i, 1) = 0;
                *rgbd.color_.PointerAt<uchar>(j, i, 2) = 0;
            }
        }
    }
    cv::Mat img = Open3DToOpenCV(rgbd.color_);
    showEdges(img);
}

void showEdges(cv::Mat img)
{
    using namespace cv;
    // GaussianBlur(img, img, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(img_gray, img_gray);
    Mat grad_y, abs_grad_y;
    cv::Sobel(img_gray, grad_y, CV_16S, 0, 1);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::addWeighted(abs_grad_y, 0.5, abs_grad_y, 0, 0, img_gray);
    cv::imshow("Test", img_gray);
    cv::waitKey(0);
}

//returns if the point p is on the ride side on the edge that goes from v1 to v2
//note: this does only work for a specific orientation of the triangles (clockwise)
// bool edgeFunction(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, const Eigen::Vector2d& p){
//     if ((p(0)-v1(0)) * (v2(1) - v1(1)) - (p(1)-v1(1)) * (v2(0)-v1(0)) <0){
//         return false;
//     } else{
//         return true;
//     }
// }

double edgeFunction(const Eigen::Vector2d &v0, const Eigen::Vector2d &v1, const Eigen::Vector2d &p)
{
    return (p(0) - v0(0)) * (v1(1) - v0(1)) - (p(1) - v0(1)) * (v1(0) - v0(0));
}

double edgeFunctionCCW(const Eigen::Vector2d &v0, const Eigen::Vector2d &v1, const Eigen::Vector2d &p)
{
    Eigen::Vector2d A = p - v0;
    Eigen::Vector2d B = v1 - v0;
    return (A(0) - B(0)) * (p(1) - A(1)) - (A(1) - B(1)) * (p(0) - A(0));
}

shared_ptr<open3d::geometry::PointCloud> EigenDepthToPCD(const Eigen::MatrixXd &depth_buffer, open3d::camera::PinholeCameraIntrinsic &intr)
{

    shared_ptr<open3d::geometry::Image> dimg = make_shared<open3d::geometry::Image>();
    dimg->Prepare(depth_buffer.cols(), depth_buffer.rows(), 1, 4);
    for (int i = 0; i < depth_buffer.rows(); i++)
    {
        for (int j = 0; j < depth_buffer.cols(); j++)
        {
            *dimg->PointerAt<float>(j, i) = depth_buffer(i, j);
        }
    }
    return open3d::geometry::PointCloud::CreateFromDepthImage(*dimg, intr);
}

std::shared_ptr<open3d::geometry::LineSet> visualizeHalfsphereVectors(const vector<Eigen::Vector3d> &vecs, const Eigen::Vector3d &p)
{

    auto ls = std::make_shared<open3d::geometry::LineSet>();
    ls->points_.push_back(p);
    for (int i = 0; i < vecs.size(); i++)
    {
        ls->points_.push_back(vecs[i]);
        ls->lines_.push_back(Eigen::Vector2i(0, i + 1));
    }
    return ls;
}

//looking into the direction of the camera, x is left, y is up
vector<Eigen::Vector3d> getHalfsphereVectors()
{
    vector<Eigen::Vector3d> out;
    double minAngle = toRadians(15);
    // double maxAngle = 2* PANDIA_PI;
    double maxAngle = PANDIA_PI - minAngle;
    double stepsxy = 5;                   //actual step size is one more
    double alphaInc = maxAngle / stepsxy; //alpha for values in x-y plane
    double stepsyz = 5;                   //actual step size is one more
    double betaInc = maxAngle / stepsyz;  //beta for yz plane

    //length of all vectors is one
    for (float alpha = minAngle; alpha <= maxAngle; alpha += alphaInc)
    {
        for (float beta = minAngle; beta <= maxAngle; beta += betaInc)
        {
            out.push_back(Eigen::Vector3d(std::sin(beta) * std::cos(alpha), std::sin(alpha) * std::sin(beta), std::cos(beta)));
        }
    }
    return out;
}

vector<Eigen::Vector3d> rotateHalfsphereVectors(const vector<Eigen::Vector3d> &vecs, const Eigen::Matrix3d &T)
{
    vector<Eigen::Vector3d> out;
    for (int i = 0; i < vecs.size(); i++)
    {
        out.push_back(T * vecs[i]);
    }
    return out;
}

//given a mesh and the associated pointcloud this function returns the part of the pcd thats acutally scannable.
shared_ptr<open3d::geometry::PointCloud> getScannablePcd(shared_ptr<open3d::geometry::TriangleMesh> mesh, shared_ptr<open3d::geometry::PointCloud> pcd)
{

    shared_ptr<open3d::geometry::PointCloud> out = make_shared<open3d::geometry::PointCloud>();
    auto basicrays = getHalfsphereVectors();
    float thres = 1e-3;
    vector<Eigen::Vector3d> points(pcd->points_.size(), Eigen::Vector3d::Zero());
    vector<Eigen::Vector3d> normals(pcd->points_.size(), Eigen::Vector3d::Zero());
    vector<Eigen::Vector3d> localPoints;
    vector<Eigen::Vector3d> localNormals;
#pragma omp parallel for
    for (int i = 0; i < pcd->points_.size(); i++)
    {
        //for every point cast rays
        //dont use straight y-axis since rodrigues will fail for parallel vectors
        //if cad model has straight y-axis aligned surface
        Eigen::Matrix3d R = getRotationMatrixFromAtoB(pcd->normals_[i], Eigen::Vector3d(0.001, 0.999, 0));
        auto rays = rotateHalfsphereVectors(basicrays, R);
        bool intersection = false;
        for (int j = 0; j < rays.size(); j++)
        {
            //for each ray iterate through all triangles
            float t = 1e6;
            for (int k = 0; k < mesh->triangles_.size(); k++)
            {
                float b, c;
                intersection = rayTriangleIntersect(pcd->points_[i], rays[j],
                                                    mesh->vertices_[mesh->triangles_[k](0)], mesh->vertices_[mesh->triangles_[k](1)],
                                                    mesh->vertices_[mesh->triangles_[k](2)], t, b, c);
                if (intersection && t < thres)
                { //threshold in case a point cuts the triangle it sits in, happens almost every time.
                    intersection = false;
                }
                if (intersection)
                {
                    break;
                }
            }
            if (!intersection) //the ray hit the sky, we do not need to check other rays for this point
            {
                break;
            }
        }

        if (!intersection)
        {
            points[i] = pcd->points_[i];
            normals[i] = pcd->normals_[i];
        }
    }
    for (int i = 0; i < points.size(); i++)
    {
        if (points[i] != Eigen::Vector3d::Zero())
        {
            out->points_.push_back(points[i]);
            out->normals_.push_back(normals[i]);
        }
    }

    return out;
}

shared_ptr<open3d::geometry::Image> EigenToO3DDepthImage(const Eigen::MatrixXd &depth)
{
    auto out = make_shared<open3d::geometry::Image>();
    out->Prepare(depth.cols(), depth.rows(), 1, 4);
    for (int i = 0; i < depth.rows(); i++)
    {
        for (int j = 0; j < depth.cols(); j++)
        {
            *out->PointerAt<float>(j, i) = depth(i, j);
        }
    }
    return out;
}

Eigen::MatrixXd O3DDepthtoEigen(shared_ptr<open3d::geometry::Image> depth)
{
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(depth->height_, depth->width_);
    for (int i = 0; i < depth->height_; i++)
    {
        for (int j = 0; j < depth->width_; j++)
        {
            out(i, j) = *depth->PointerAt<float>(j, i);
        }
    }
    return out;
}

Json::Value StringToJson(const std::string &json_str)
{
    Json::Value json;
    std::string err;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(json_str.c_str(), json_str.c_str() + json_str.length(),
                       &json, &err))
    {
        cout << "Failed to parse string to json: " << err << endl;
    }
    return json;
}

std::string JsonToString(const Json::Value json)
{
    return Json::writeString(Json::StreamWriterBuilder(), json);
}

cv::Matx33d eigenToOpenCv33(const Eigen::Matrix3d &m)
{
    cv::Matx33d out;
    out(0, 0) = m(0, 0);
    out(0, 1) = m(0, 1);
    out(0, 2) = m(0, 2);
    out(1, 0) = m(1, 0);
    out(1, 1) = m(1, 1);
    out(1, 2) = m(1, 2);
    out(2, 0) = m(2, 0);
    out(2, 1) = m(2, 1);
    out(2, 2) = m(2, 1);
    return out;
}

// The input 3D points are stored as columns.
Eigen::Matrix3d kabsch(const Eigen::Matrix3Xd &source, const Eigen::Matrix3Xd &target)
{

    if (source.cols() != target.cols())
        throw "Find3DAffineTransform(): input data mis-match";

    // SVD
    Eigen::MatrixXd Cov = source * target.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Find the rotation
    double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    if (d > 0) //why?
        d = 1.0;
    else
        d = -1.0;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    I(2, 2) = d;
    Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

    return R;
}

bool IsDepthImageValid(const open3d::geometry::Image &depth)
{
    int nInvalidPixels = 0;
#pragma omp parallel for reduction(+ \
                                   : nInvalidPixels)
    for (int j = 0; j < depth.height_; j++)
    {
        for (int k = 0; k < depth.width_; k++)
        {
            //note the coordinate change
            float d = *depth.PointerAt<float>(k, j);
            if (d <= 0 || d > 1e6)
            {
                nInvalidPixels++;
            }
        }
    }
    if (nInvalidPixels > 0.75 * depth.height_ * depth.width_)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// for frameset with color, depth and infrared frame
bool IsIntelFramesetValid(const rs2::frameset &fs)
{
    bool valid = true;
    if (!fs.get_color_frame())
    {
        // cout << "Warning: Color frame is invalid!" << endl;
        valid = false;
    }
    if (!fs.get_depth_frame())
    {
        // cout << "Warning: Depth frame is invalid!" << endl;
        valid = false;
    }
    if (!fs.get_infrared_frame())
    {
        // cout << "Warning: Infrared frame is invalid!" << endl;
        valid = false;
    }
    return valid;
}

std::string getExtensionFromFilename(const std::string &filename)
{
    size_t pos = filename.find_last_of(".");
    if (pos != string::npos)
    {
        return filename.substr(pos + 1);
    }
    else
    {
        cout << "Warning: getExtensionFromFilename failed." << endl;
        return "";
    }
}

open3d::camera::PinholeCameraIntrinsic getScaledIntrinsic(const open3d::camera::PinholeCameraIntrinsic &intr, int newWidth, int newHeight)
{
    double factorWidth = (double)newWidth / (double)intr.width_;
    double factorHeight = (double)newHeight / (double)intr.height_;
    if (factorWidth != factorHeight)
    {
        cout << "Warning: scaled intrinsic has different aspect ratio" << endl;
    }
    double fx = intr.intrinsic_matrix_(0, 0) * factorWidth;
    double fy = intr.intrinsic_matrix_(1, 1) * factorHeight;
    double cx = intr.intrinsic_matrix_(0, 2) * factorWidth;
    double cy = intr.intrinsic_matrix_(1, 2) * factorHeight;
    return open3d::camera::PinholeCameraIntrinsic(newWidth, newHeight, fx, fy, cx, cy);
}
