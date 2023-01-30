#include "guiUtil.h"
#include "CameraWrapper.h"
#include "util.h"
using namespace std;
void o3dToOgreMesh(Ogre::ManualObject *mesh, std::shared_ptr<open3d::geometry::TriangleMesh> o3dMesh,
const string& MaterialName, const Eigen::Matrix3d& trans, float alpha)
{
    mesh->clear();
    mesh->estimateVertexCount(o3dMesh->vertices_.size());
    mesh->estimateIndexCount(o3dMesh->triangles_.size());
    mesh->begin(MaterialName, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    //########### case no color and no normal ##############
    if (!o3dMesh->HasVertexNormals() && !o3dMesh->HasVertexColors())
    {
        for (int i = 0; i < o3dMesh->vertices_.size(); i++)
        {
            Eigen::Vector3d p = trans * o3dMesh->vertices_[i];
            mesh->position(p(0), p(1), p(2));
            mesh->colour(0.5, 0.5, 0.5, alpha);
            
        }
    }
    //########## case normal but no color ###############
    if (o3dMesh->HasVertexNormals() && !o3dMesh->HasVertexColors())
    {
        for (int i = 0; i < o3dMesh->vertices_.size(); i++)
        {
            Eigen::Vector3d p = trans * o3dMesh->vertices_[i];
            Eigen::Vector3d n = trans * o3dMesh->vertex_normals_[i];
            mesh->position(p(0), p(1), p(2));
            mesh->normal(n(0), n(1), n(2));
            mesh->colour(0.5, 0.5, 0.5, alpha);
            if (o3dMesh->HasTriangleUvs()) // only relevant for camermodels atm
            {
                Eigen::Vector2d uv = o3dMesh->triangle_uvs_.at(i);
                mesh->textureCoord(uv(0), 1-uv(1));
            }
        }
    }

    //########## case no normals but has color ###############
    if (!o3dMesh->HasVertexNormals() && o3dMesh->HasVertexColors())
    {
        for (int i = 0; i < o3dMesh->vertices_.size(); i++)
        {
            Eigen::Vector3d p = trans * o3dMesh->vertices_[i];
            Eigen::Vector3d &c = o3dMesh->vertex_colors_[i];
            mesh->position(p(0), p(1), p(2));
            mesh->colour(c(0), c(1), c(2), alpha);
        }
    }

    //########## case normal and color ###############
    if (o3dMesh->HasVertexNormals() && o3dMesh->HasVertexColors())
    {
        for (int i = 0; i < o3dMesh->vertices_.size(); i++)
        {
            Eigen::Vector3d p = trans * o3dMesh->vertices_[i];
            Eigen::Vector3d n = trans * o3dMesh->vertex_normals_[i];
            Eigen::Vector3d &c = o3dMesh->vertex_colors_[i];
            mesh->position(p(0), p(1), p(2));
            mesh->normal(n(0), n(1), n(2));
            mesh->colour(c(0), c(1), c(2), alpha);
        }
    }
    for (auto &index : o3dMesh->triangles_)
    {
        mesh->triangle(index(0), index(1), index(2));
    }
    mesh->end(); // creates the actual hardware buffers to be used for rendering
}

void o3dToOgrePcd(Ogre::ManualObject *pcd, std::shared_ptr<open3d::geometry::PointCloud> o3dpcd, 
const string &MaterialName, const Eigen::Matrix3d& trans, float alpha)
{
    pcd->clear();
    pcd->estimateVertexCount(o3dpcd->points_.size());
    pcd->begin(MaterialName, Ogre::RenderOperation::OT_POINT_LIST);

    for (int i = 0; i < o3dpcd->points_.size(); i++)
    {
        Eigen::Vector3d p = trans * o3dpcd->points_[i];
        pcd->position(p(0), p(1), p(2));
        if (o3dpcd->HasNormals())
        {
            Eigen::Vector3d n = trans * o3dpcd->normals_[i];
            pcd->normal(n(0), n(1), n(2));
        }
        if (o3dpcd->HasColors())
        {
            Eigen::Vector3d &c = o3dpcd->colors_[i];
            pcd->colour(c(0), c(1), c(2), alpha);
        }
        else 
        {
            pcd->colour(0.5, 0.5, 0.5, alpha);
        }
    }
    pcd->end(); // creates the actual hardware buffers to be used for rendering
}

void o3dToOgreLineList(Ogre::ManualObject *mesh, std::shared_ptr<open3d::geometry::LineSet> o3dLineList,
                       const string &MaterialName, const Eigen::Matrix3d &trans, float alpha)
{
    mesh->clear();
    mesh->estimateVertexCount(o3dLineList->points_.size());
    mesh->estimateIndexCount(o3dLineList->lines_.size());
    mesh->begin(MaterialName, Ogre::RenderOperation::OT_LINE_LIST);
    //########### case no color##############
    if (!o3dLineList->HasColors())
    {
        for (int i = 0; i < o3dLineList->points_.size(); i++)
        {
            Eigen::Vector3d p = trans * o3dLineList->points_[i];
            mesh->position(p(0), p(1), p(2));
            mesh->colour(0.5, 0.5, 0.5, alpha);
        }
    }
    //########## case color ###############
    if (o3dLineList->HasColors())
    {
        for (int i = 0; i < o3dLineList->points_.size(); i++)
        {
            Eigen::Vector3d p = trans * o3dLineList->points_[i];
            Eigen::Vector3d &c = o3dLineList->colors_[i];
            mesh->position(p(0), p(1), p(2));
            mesh->colour(c(0), c(1), c(2), alpha);
        }
    }
    for (auto &index : o3dLineList->lines_)
    {
        mesh->index(index(0));
        mesh->index(index(1));
    }
    mesh->end(); // creates the actual hardware buffers to be used for rendering
}

std::shared_ptr<open3d::geometry::TriangleMesh> readMesh(const string &path)
{
    auto out = std::make_shared<open3d::geometry::TriangleMesh>();
    if (open3d::io::ReadTriangleMesh(path, *out))
    {
        return out;
    }
    else
    {
        cout << "warning: readMesh failed, using empty mesh!" << endl;
        return std::make_shared<open3d::geometry::TriangleMesh>();
    }
}

std::shared_ptr<open3d::geometry::PointCloud> readPcd(const string &path)
{
    auto out = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloudOption params;
    if (open3d::io::ReadPointCloudFromPLY(path, *out,params))
    {
        return out;
    }
    else
    {
        cout << "warning: reading Pointcloud failed!" << endl;
        return std::make_shared<open3d::geometry::PointCloud>();
    }
}

// fill vector of model paths
std::vector<std::string> getFileNames(const std::string& folder)
{
    if (folder.empty())
    {
        cout << "warning: no model directory supplied" << endl;
    }
    string path = get_current_dir_name() + folder;
    std::vector<std::string> out;

    // dirent.h file handling to get files in directory
    DIR *dir;
    struct dirent *file;
    if ((dir = opendir(path.c_str())) != NULL)
    {
        /* print all the files and directories within directory */
        while ((file = readdir(dir)) != NULL)
        {
            if (!file->d_name || file->d_name[0] == '.')
            {
                continue; // skip everything that starts with a dot
            }
            // printf("%s\n", file->d_name);
            out.push_back(file->d_name);
        }
        closedir(dir);
    }
    else
    {
        cout << "warning: could not open directory: " << path << endl;
    }
    std::sort(out.begin(), out.end(), SortbyAlphabet);
    return out;
}

//return a vector containing the Camera Serial Numbers
std::vector<std::string> getCameraSerialNumbers(std::shared_ptr<std::vector<CameraWrapper>> cameras)
{
    std::vector<std::string> out;
    for (auto &c : *cameras)
    {
        out.push_back(c.SerialNumber);
    }
    return out;
}

Ogre::Vector3 o3dToOgre(const Eigen::Vector3d &in)
{

    return Ogre::Vector3(in(0),in(1),in(2));
}

//no header
void DeleteDuplicateVertices(shared_ptr<open3d::geometry::TriangleMesh> in)
{
    for (int i = 0; i < in->vertices_.size(); i++)
    {
        if (i >= in->vertices_.size())
            break;
        for (int j = 0; j < in->vertices_.size(); j++)
        {
            if (j >= in->vertices_.size() || i >= in->vertices_.size())
                break;
            if (in->vertices_.at(i).x() < in->vertices_.at(j).x() + 1e-6 && in->vertices_.at(i).x() > in->vertices_.at(j).x() - 1e-6 && in->vertices_.at(i).y() < in->vertices_.at(j).y() + 1e-6 && in->vertices_.at(i).y() > in->vertices_.at(j).y() - 1e-6 && in->vertices_.at(i).z() < in->vertices_.at(j).z() + 1e-6 && in->vertices_.at(i).z() > in->vertices_.at(j).z() - 1e-6 && i != j)
            {
                in->vertices_.erase(in->vertices_.begin() + j);
            }
        }
    }
}

//also fills lines with the wired geometry
std::shared_ptr<open3d::geometry::TriangleMesh> getFOVGeometryfromPlanes(shared_ptr<std::vector<PandiaPlane>> planes, std::shared_ptr<open3d::geometry::LineSet> lines)
{
    std::shared_ptr<std::vector<PandiaRay>> rays = make_shared<vector<PandiaRay>>();
    std::shared_ptr<std::vector<Eigen::Vector3d>> verts = make_shared<vector<Eigen::Vector3d>>();
    auto out = make_shared<open3d::geometry::TriangleMesh>();
    for (auto &p1 : *planes)
    {
        for (auto &p2 : *planes)
        {
            if (p1.PlanePlaneIntersection(p2).Direction != Eigen::Vector3d::Zero())
            {
                rays->push_back(p1.PlanePlaneIntersection(p2));
            }
        }
    }
    for (auto &r1 : *rays)
    {
        for (auto &p : *planes)
        {
            if (r1.RayPlaneIntersects(p))
            {
                verts->push_back(r1.RayPlaneIntersection(p));
            }
        }
    }

    for (auto &p : *planes)
    {
        for (int i = 0; i < verts->size(); i++)
        {

            if ((verts->at(i) - p.d * p.n).dot(p.n) > 1e-6)
            {
                verts->erase(verts->begin() + i);
                i--;
            }
        }
    }

    for (auto &v1 : *verts)
    {
        out->vertices_.push_back(v1);
    }
    for (int i = 0; i < out->vertices_.size(); i++)
    {
        if (std::abs(out->vertices_.at(i).x()) < 1e-6)
            out->vertices_.at(i).x() = 0;
        if (std::abs(out->vertices_.at(i).y()) < 1e-6)
            out->vertices_.at(i).y() = 0;
        if (std::abs(out->vertices_.at(i).z()) < 1e-6)
            out->vertices_.at(i).z() = 0;
    }
    DeleteDuplicateVertices(out);

    std::vector<Eigen::Vector3d> edges;
    bool talreadyexists = false;
    bool dont = false;
    int count = 0;
    int planecount = 0;
    bool dobreak = false;
    for (int i = 0; i < out->vertices_.size(); i++)
    {
        for (int j = 0; j < out->vertices_.size(); j++)
        {
            for (int k = 0; k < out->vertices_.size(); k++)
            {
                for (auto &p : *planes)
                {
                    for (auto t : out->triangles_)
                    {
                        if (t == Eigen::Vector3i(i, j, k) || t == Eigen::Vector3i(j, k, i) || t == Eigen::Vector3i(k, i, j) || t == Eigen::Vector3i(i, k, j) || t == Eigen::Vector3i(j, i, k) || t == Eigen::Vector3i(k, j, i))
                            talreadyexists = true;
                    }
                    bool makebreak = true;
                    bool foundplane;
                    if (out->triangle_normals_.size() > 0)
                    {
                        foundplane = false;
                        for (int tn = 0; tn < out->triangle_normals_.size(); tn++)
                        {
                            if (((out->vertices_.at(j) - out->vertices_.at(i)).cross(out->vertices_.at(k) - out->vertices_.at(i))).normalized().dot(out->triangle_normals_.at(tn).normalized()) > 1 - 1e-6 && ((out->vertices_.at(j) - out->vertices_.at(i)).cross(out->vertices_.at(k) - out->vertices_.at(i))).normalized().dot(out->triangle_normals_.at(tn).normalized()) < 1 + 1e-6)
                            {
                                foundplane = true;
                                if (out->triangles_.at(tn).x() == i && out->triangles_.at(tn).y() == j || out->triangles_.at(tn).x() == i && out->triangles_.at(tn).z() == j || out->triangles_.at(tn).y() == i && out->triangles_.at(tn).x() == j || out->triangles_.at(tn).y() == i && out->triangles_.at(tn).z() == j || out->triangles_.at(tn).z() == i && out->triangles_.at(tn).x() == j || out->triangles_.at(tn).z() == i && out->triangles_.at(tn).y() == j || out->triangles_.at(tn).x() == j && out->triangles_.at(tn).y() == k || out->triangles_.at(tn).x() == j && out->triangles_.at(tn).z() == k || out->triangles_.at(tn).y() == j && out->triangles_.at(tn).x() == k || out->triangles_.at(tn).y() == j && out->triangles_.at(tn).z() == k || out->triangles_.at(tn).z() == j && out->triangles_.at(tn).x() == k || out->triangles_.at(tn).z() == j && out->triangles_.at(tn).y() == k || out->triangles_.at(tn).x() == i && out->triangles_.at(tn).y() == k || out->triangles_.at(tn).x() == i && out->triangles_.at(tn).z() == k || out->triangles_.at(tn).y() == i && out->triangles_.at(tn).x() == k || out->triangles_.at(tn).y() == i && out->triangles_.at(tn).z() == k || out->triangles_.at(tn).z() == i && out->triangles_.at(tn).x() == k || out->triangles_.at(tn).z() == i && out->triangles_.at(tn).y() == k)
                                {
                                    makebreak = false;
                                    break;
                                }
                            }
                        }
                    }
                    else
                    {
                        makebreak = false;
                    }
                    if (!foundplane)
                    {
                        makebreak = false;
                    }

                    if (makebreak)
                    {
                        break;
                    }
                    if (((out->vertices_.at(j) - out->vertices_.at(i)).cross(out->vertices_.at(k) - out->vertices_.at(i))).normalized().dot(p.n) > 1 - 1e-6 && p.n.dot(out->vertices_.at(i) - p.d * p.n) >= -1e-6 && ((out->vertices_.at(j) - out->vertices_.at(i)).cross(out->vertices_.at(k) - out->vertices_.at(i))).normalized().dot(p.n) < 1 + 1e-6 && !talreadyexists)
                    {
                        if (out->triangle_normals_.size() < 1)
                        {
                            out->triangles_.push_back(Eigen::Vector3i(i, j, k));
                            out->triangle_normals_.push_back(p.n);
                        }
                        else
                        {

                            for (int old = 0; old < out->triangle_normals_.size(); old++)
                            {
                                if (((out->vertices_.at(j) - out->vertices_.at(i)).cross(out->vertices_.at(k) - out->vertices_.at(i))).normalized().dot(out->triangle_normals_.at(old).normalized()) > 1 - 1e-6 && ((out->vertices_.at(j) - out->vertices_.at(i)).cross(out->vertices_.at(k) - out->vertices_.at(i))).normalized().dot(out->triangle_normals_.at(old).normalized()) < 1 + 1e-6)
                                {

                                    if (out->triangles_.at(old).x() == i && out->triangles_.at(old).y() == j || out->triangles_.at(old).x() == i && out->triangles_.at(old).z() == j || out->triangles_.at(old).y() == i && out->triangles_.at(old).x() == j || out->triangles_.at(old).y() == i && out->triangles_.at(old).z() == j || out->triangles_.at(old).z() == i && out->triangles_.at(old).x() == j || out->triangles_.at(old).z() == i && out->triangles_.at(old).y() == j)
                                    {
                                        for (auto &r : *rays)
                                        {
                                            Eigen::Vector3d lineDirection1 = (out->vertices_.at(i) - out->vertices_.at(j)).normalized();
                                            Eigen::Vector3d lineDirection2 = (out->vertices_.at(j) - out->vertices_.at(i)).normalized();
                                            if (lineDirection1.x() < 1e-6 && lineDirection1.x() > -1e-6)
                                                lineDirection1.x() = 0;
                                            if (lineDirection1.y() < 1e-6 && lineDirection1.y() > -1e-6)
                                                lineDirection1.y() = 0;
                                            if (lineDirection1.z() < 1e-6 && lineDirection1.z() > -1e-6)
                                                lineDirection1.z() = 0;
                                            if (lineDirection2.x() < 1e-6 && lineDirection2.x() > -1e-6)
                                                lineDirection2.x() = 0;
                                            if (lineDirection2.y() < 1e-6 && lineDirection2.y() > -1e-6)
                                                lineDirection2.y() = 0;
                                            if (lineDirection2.z() < 1e-6 && lineDirection2.z() > -1e-6)
                                                lineDirection2.z() = 0;
                                            if (lineDirection1.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection1.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection1.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection1.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection1.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection1.normalized().z() >= r.Direction.normalized().z() - 1e-6 || lineDirection2.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection2.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection2.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection2.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection2.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection2.normalized().z() >= r.Direction.normalized().z() - 1e-6)
                                            {
                                                dont = true;
                                                dobreak = true;
                                                break;
                                            }
                                            else
                                            {
                                                int middlecount = 0;
                                                bool pointsOnDifferentSides = true;
                                                for (auto f : out->triangles_)
                                                {
                                                    Eigen::Vector3d trianglePoint;
                                                    if (f.x() == i && f.y() == j || f.x() == i && f.z() == j || f.y() == i && f.x() == j || f.y() == i && f.z() == j || f.z() == i && f.x() == j || f.z() == i && f.y() == j)
                                                    {
                                                        middlecount++;
                                                        if (f.x() == i && f.y() == j || f.x() == j && f.y() == i)
                                                            trianglePoint = out->vertices_.at(f.z());
                                                        if (f.x() == i && f.z() == j || f.x() == j && f.z() == i)
                                                            trianglePoint = out->vertices_.at(f.y());
                                                        if (f.y() == i && f.z() == j || f.y() == j && f.z() == i)
                                                            trianglePoint = out->vertices_.at(f.x());

                                                        if ((out->vertices_.at(k) - out->vertices_.at(i)).dot(lineDirection2.cross(p.n).normalized()) > 0 && (trianglePoint - out->vertices_.at(i)).dot(lineDirection2.cross(p.n).normalized()) > 0 || (out->vertices_.at(k) - out->vertices_.at(i)).dot(lineDirection2.cross(p.n).normalized()) < 0 && (trianglePoint - out->vertices_.at(i)).dot(lineDirection2.cross(p.n).normalized()) < 0)
                                                        {
                                                            pointsOnDifferentSides = false;
                                                        }
                                                    }
                                                }
                                                if (middlecount >= 2 || !pointsOnDifferentSides)
                                                {
                                                    dont = true;
                                                    pointsOnDifferentSides = true;
                                                    dobreak = true;
                                                    break;
                                                }
                                            }
                                            if (dobreak)
                                            {
                                                dobreak = false;
                                                break;
                                            }
                                        }
                                        if (!dont)
                                        {

                                            out->triangles_.push_back(Eigen::Vector3i(i, j, k));
                                            out->triangle_normals_.push_back(p.n);
                                            dont = false;
                                            break;
                                        }
                                    }
                                    else if (out->triangles_.at(old).x() == j && out->triangles_.at(old).y() == k || out->triangles_.at(old).x() == j && out->triangles_.at(old).z() == k || out->triangles_.at(old).y() == j && out->triangles_.at(old).x() == k || out->triangles_.at(old).y() == j && out->triangles_.at(old).z() == k || out->triangles_.at(old).z() == j && out->triangles_.at(old).x() == k || out->triangles_.at(old).z() == j && out->triangles_.at(old).y() == k)
                                    {
                                        for (auto &r : *rays)
                                        {
                                            Eigen::Vector3d lineDirection1 = (out->vertices_.at(j) - out->vertices_.at(k)).normalized();
                                            Eigen::Vector3d lineDirection2 = (out->vertices_.at(k) - out->vertices_.at(j)).normalized();
                                            if (lineDirection1.x() < 1e-6 && lineDirection1.x() > -1e-6)
                                                lineDirection1.x() = 0;
                                            if (lineDirection1.y() < 1e-6 && lineDirection1.y() > -1e-6)
                                                lineDirection1.y() = 0;
                                            if (lineDirection1.z() < 1e-6 && lineDirection1.z() > -1e-6)
                                                lineDirection1.z() = 0;
                                            if (lineDirection2.x() < 1e-6 && lineDirection2.x() > -1e-6)
                                                lineDirection2.x() = 0;
                                            if (lineDirection2.y() < 1e-6 && lineDirection2.y() > -1e-6)
                                                lineDirection2.y() = 0;
                                            if (lineDirection2.z() < 1e-6 && lineDirection2.z() > -1e-6)
                                                lineDirection2.z() = 0;
                                            if (lineDirection1.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection1.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection1.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection1.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection1.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection1.normalized().z() >= r.Direction.normalized().z() - 1e-6 || lineDirection2.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection2.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection2.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection2.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection2.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection2.normalized().z() >= r.Direction.normalized().z() - 1e-6)
                                            {
                                                dont = true;
                                                dobreak = true;
                                                break;
                                            }
                                            else
                                            {
                                                int middlecount = 0;
                                                bool pointsOnDifferentSides = true;
                                                for (auto f : out->triangles_)
                                                {
                                                    Eigen::Vector3d trianglePoint;
                                                    if (f.x() == j && f.y() == k || f.x() == j && f.z() == k || f.y() == j && f.x() == k || f.y() == j && f.z() == k || f.z() == j && f.x() == k || f.z() == j && f.y() == k)
                                                    {
                                                        middlecount++;
                                                        if (f.x() == j && f.y() == k || f.x() == k && f.y() == j)
                                                            trianglePoint = out->vertices_.at(f.z());
                                                        if (f.x() == j && f.z() == k || f.x() == k && f.z() == j)
                                                            trianglePoint = out->vertices_.at(f.y());
                                                        if (f.y() == j && f.z() == k || f.y() == k && f.z() == j)
                                                            trianglePoint = out->vertices_.at(f.x());

                                                        if ((out->vertices_.at(i) - out->vertices_.at(k)).dot(lineDirection1.cross(p.n).normalized()) > 0 && (trianglePoint - out->vertices_.at(k)).dot(lineDirection1.cross(p.n).normalized()) > 0 || (out->vertices_.at(i) - out->vertices_.at(k)).dot(lineDirection1.cross(p.n).normalized()) < 0 && (trianglePoint - out->vertices_.at(k)).dot(lineDirection1.cross(p.n).normalized()) < 0)
                                                        {
                                                            pointsOnDifferentSides = false;
                                                        }
                                                    }
                                                }
                                                if (middlecount >= 2 || !pointsOnDifferentSides)
                                                {
                                                    dont = true;
                                                    pointsOnDifferentSides = true;
                                                    dobreak = true;
                                                    break;
                                                }
                                            }
                                            if (dobreak)
                                            {
                                                dobreak = false;
                                                break;
                                            }
                                        }
                                        if (!dont)
                                        {
                                            out->triangles_.push_back(Eigen::Vector3i(i, j, k));
                                            out->triangle_normals_.push_back(p.n);
                                            dont = false;
                                            break;
                                        }
                                    }
                                    else if (out->triangles_.at(old).x() == i && out->triangles_.at(old).y() == k || out->triangles_.at(old).x() == i && out->triangles_.at(old).z() == k || out->triangles_.at(old).y() == i && out->triangles_.at(old).x() == k || out->triangles_.at(old).y() == i && out->triangles_.at(old).z() == k || out->triangles_.at(old).z() == i && out->triangles_.at(old).x() == k || out->triangles_.at(old).z() == i && out->triangles_.at(old).y() == k)
                                    {
                                        for (auto &r : *rays)
                                        {
                                            Eigen::Vector3d lineDirection1 = (out->vertices_.at(i) - out->vertices_.at(k)).normalized();
                                            Eigen::Vector3d lineDirection2 = (out->vertices_.at(k) - out->vertices_.at(i)).normalized();
                                            if (lineDirection1.x() < 1e-6 && lineDirection1.x() > -1e-6)
                                                lineDirection1.x() = 0;
                                            if (lineDirection1.y() < 1e-6 && lineDirection1.y() > -1e-6)
                                                lineDirection1.y() = 0;
                                            if (lineDirection1.z() < 1e-6 && lineDirection1.z() > -1e-6)
                                                lineDirection1.z() = 0;
                                            if (lineDirection2.x() < 1e-6 && lineDirection2.x() > -1e-6)
                                                lineDirection2.x() = 0;
                                            if (lineDirection2.y() < 1e-6 && lineDirection2.y() > -1e-6)
                                                lineDirection2.y() = 0;
                                            if (lineDirection2.z() < 1e-6 && lineDirection2.z() > -1e-6)
                                                lineDirection2.z() = 0;
                                            if (lineDirection1.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection1.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection1.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection1.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection1.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection1.normalized().z() >= r.Direction.normalized().z() - 1e-6 || lineDirection2.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection2.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection2.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection2.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection2.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection2.normalized().z() >= r.Direction.normalized().z() - 1e-6)
                                            {
                                                dont = true;
                                                dobreak = true;
                                                break;
                                            }
                                            else
                                            {
                                                int middlecount = 0;
                                                bool pointsOnDifferentSides = true;
                                                for (auto f : out->triangles_)
                                                {
                                                    Eigen::Vector3d trianglePoint;
                                                    if (f.x() == i && f.y() == k || f.x() == i && f.z() == k || f.y() == i && f.x() == k || f.y() == i && f.z() == k || f.z() == i && f.x() == k || f.z() == i && f.y() == k)
                                                    {
                                                        middlecount++;
                                                        if (f.x() == i && f.y() == k || f.x() == k && f.y() == i)
                                                            trianglePoint = out->vertices_.at(f.z());
                                                        if (f.x() == i && f.z() == k || f.x() == k && f.z() == i)
                                                            trianglePoint = out->vertices_.at(f.y());
                                                        if (f.y() == i && f.z() == k || f.y() == k && f.z() == i)
                                                            trianglePoint = out->vertices_.at(f.x());

                                                        if ((out->vertices_.at(j) - out->vertices_.at(i)).dot(lineDirection1.cross(p.n).normalized()) > 0 && (trianglePoint - out->vertices_.at(i)).dot(lineDirection1.cross(p.n).normalized()) > 0 || (out->vertices_.at(j) - out->vertices_.at(i)).dot(lineDirection1.cross(p.n).normalized()) < 0 && (trianglePoint - out->vertices_.at(i)).dot(lineDirection1.cross(p.n).normalized()) < 0)
                                                        {
                                                            pointsOnDifferentSides = false;
                                                        }
                                                    }
                                                }
                                                if (middlecount >= 2 || !pointsOnDifferentSides)
                                                {
                                                    dont = true;
                                                    pointsOnDifferentSides = true;
                                                    dobreak = true;
                                                    break;
                                                }
                                            }
                                            if (dobreak)
                                            {
                                                dobreak = false;
                                                break;
                                            }
                                        }
                                        if (!dont)
                                        {
                                            out->triangles_.push_back(Eigen::Vector3i(i, j, k));
                                            out->triangle_normals_.push_back(p.n);
                                            dont = false;
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        count++;
                                    }
                                }
                                dont = false;
                                dobreak = false;
                            }

                            for (auto t : out->triangle_normals_)
                            {
                                if (t.normalized().x() <= p.n.x() + 1e-6 && t.normalized().x() >= p.n.x() - 1e-6 && t.normalized().y() <= p.n.y() + 1e-6 && t.normalized().y() >= p.n.y() - 1e-6 && t.normalized().z() <= p.n.z() + 1e-6 && t.normalized().z() >= p.n.z() - 1e-6)
                                {
                                    planecount++;
                                }
                            }
                            if (planecount == count)
                            {
                                out->triangles_.push_back(Eigen::Vector3i(i, j, k));
                                out->triangle_normals_.push_back(p.n);
                                count = 0;
                                planecount = 0;
                            }
                        }
                    }
                    talreadyexists = false;
                    count = 0;
                    planecount = 0;
                }
            }
        }
    }
    lines->Clear();
    lines->points_ = out->vertices_;
    for (int i = 0; i < lines->points_.size(); i++)
    {
        for (int j = 0; j < lines->points_.size(); j++)
        {
            Eigen::Vector3d lineDirection1 = (lines->points_.at(i) - lines->points_.at(j)).normalized();
            Eigen::Vector3d lineDirection2 = (lines->points_.at(j) - lines->points_.at(i)).normalized();
            if (lineDirection1.x() < 1e-6 && lineDirection1.x() > -1e-6)
                lineDirection1.x() = 0;
            if (lineDirection1.y() < 1e-6 && lineDirection1.y() > -1e-6)
                lineDirection1.y() = 0;
            if (lineDirection1.z() < 1e-6 && lineDirection1.z() > -1e-6)
                lineDirection1.z() = 0;
            if (lineDirection2.x() < 1e-6 && lineDirection2.x() > -1e-6)
                lineDirection2.x() = 0;
            if (lineDirection2.y() < 1e-6 && lineDirection2.y() > -1e-6)
                lineDirection2.y() = 0;
            if (lineDirection2.z() < 1e-6 && lineDirection2.z() > -1e-6)
                lineDirection2.z() = 0;
            for (auto &r : *rays)
            {
                if (lineDirection1.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection1.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection1.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection1.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection1.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection1.normalized().z() >= r.Direction.normalized().z() - 1e-6 || lineDirection2.x() <= r.Direction.normalized().x() + 1e-6 && lineDirection2.normalized().x() >= r.Direction.normalized().x() - 1e-6 && lineDirection2.y() <= r.Direction.normalized().y() + 1e-6 && lineDirection2.normalized().y() >= r.Direction.normalized().y() - 1e-6 && lineDirection2.z() <= r.Direction.normalized().z() + 1e-6 && lineDirection2.normalized().z() >= r.Direction.normalized().z() - 1e-6)
                {
                    lines->lines_.push_back(Eigen::Vector2i(i, j));
                }
            }
        }
    }
    lines->PaintUniformColor(Eigen::Vector3d(1, 0, 1));
    out->ComputeTriangleNormals();
    out->PaintUniformColor(Eigen::Vector3d(0.1, 0.1, 0.1));
    lines->PaintUniformColor(Eigen::Vector3d(0, 0, 0));
    return out;
}

