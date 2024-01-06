#include "corridor.h"

// #define PRINT

std::vector<Eigen::MatrixXd> hPolys_;

std::vector<Eigen::MatrixXd> getRectangle(std::vector<Eigen::Vector3d> statelist)
{
    hPolys_.clear();
    double resolution = 0.1;
    double step = resolution * 1.0;
    double limitBound = 2;
    // generate a rectangle for this state px py yaw
    int index = 0;
    std::cout << "statelist.size():" << statelist.size() << std::endl;
    for (const auto state : statelist)
    {
        // generate a hPoly
        Eigen::MatrixXd hPoly;
        hPoly.resize(4, 4);
        Eigen::Matrix<int, 4, 1> NotFinishTable = Eigen::Matrix<int, 4, 1>(1, 1, 1, 1);
        Eigen::Vector2d sourcePt = state.head(2); // x,y
        Eigen::Vector2d rawPt = sourcePt;         // x,y
#ifdef PRINT
        std::cout << "rawPt[" << index << "]:" << rawPt[0] << "," << rawPt[1] << std::endl;
#endif
        double yaw = state[2];
        bool test = false;
        VehicleParam_ sourceVp, rawVp;
        Eigen::Matrix2d egoR;
        egoR << cos(yaw), -sin(yaw),
            sin(yaw), cos(yaw);
        VehicleParam_ vptest;
        Eigen::Vector4d expandLength;
        expandLength << 0.0, 0.0, 0.0, 0.0;
        // dcr width length
        while (NotFinishTable.norm() > 0)
        {
            //+dy  +dx -dy -dx
            for (int i = 0; i < 4; i++)
            {
                if (!NotFinishTable[i])
                    continue;
                // get the new source and vp
                Eigen::Vector2d NewsourcePt = sourcePt;
                // common::VehicleParam NewsourceVp = sourceVp;
                VehicleParam_ NewsourceVp = sourceVp;
                Eigen::Vector2d point1, point2, newpoint1, newpoint2;

                bool isocc = false;
                switch (i)
                {
                //+dy
                case 0:
                    point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(), sourceVp.width() / 2.0);            // LF
                    point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(), sourceVp.width() / 2.0);           // LB
                    newpoint1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(), sourceVp.width() / 2.0 + step);  //+dy
                    newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(), sourceVp.width() / 2.0 + step); //+dy
                    // 1 new1 new1 new2 new2 2
                    // map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
                    CheckIfCollisionUsingLine(point1, newpoint1, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    // map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
                    CheckIfCollisionUsingLine(newpoint1, newpoint2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    CheckIfCollisionUsingLine(newpoint2, point2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    expandLength[i] += step;
                    if (expandLength[i] >= limitBound)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0, step / 2.0);
                    NewsourceVp.set_width(NewsourceVp.width() + step);
                    sourcePt = NewsourcePt;
                    sourceVp = NewsourceVp;
                    break;
                //+dx
                case 1:
                    point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(), -sourceVp.width() / 2.0);
                    point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(), sourceVp.width() / 2.0);
                    newpoint1 = sourcePt + egoR * Eigen::Vector2d(step + sourceVp.length() / 2.0 + sourceVp.d_cr(), -sourceVp.width() / 2.0);
                    newpoint2 = sourcePt + egoR * Eigen::Vector2d(step + sourceVp.length() / 2.0 + sourceVp.d_cr(), sourceVp.width() / 2.0);
                    // 1 new1 new1 new2 new2 2
                    CheckIfCollisionUsingLine(point1, newpoint1, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    CheckIfCollisionUsingLine(newpoint1, newpoint2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    CheckIfCollisionUsingLine(newpoint2, point2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    expandLength[i] += step;
                    if (expandLength[i] >= limitBound)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(step / 2.0, 0.0);
                    NewsourceVp.set_length(NewsourceVp.length() + step);
                    sourcePt = NewsourcePt;
                    sourceVp = NewsourceVp;
                    break;
                //-dy
                case 2:
                    point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(), -sourceVp.width() / 2.0);
                    point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(), -sourceVp.width() / 2.0);
                    newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(), -sourceVp.width() / 2.0 - step);
                    newpoint2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(), -sourceVp.width() / 2.0 - step);
                    // 1 new1 new1 new2 new2 2
                    CheckIfCollisionUsingLine(point1, newpoint1, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    CheckIfCollisionUsingLine(newpoint1, newpoint2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    CheckIfCollisionUsingLine(newpoint2, point2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    expandLength[i] += step;
                    if (expandLength[i] >= limitBound)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0, -step / 2.0);
                    NewsourceVp.set_width(NewsourceVp.width() + step);
                    sourcePt = NewsourcePt;
                    sourceVp = NewsourceVp;
                    break;
                //-dx
                case 3:
                    point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(), sourceVp.width() / 2.0);
                    point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(), -sourceVp.width() / 2.0);
                    newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr() - step, sourceVp.width() / 2.0);
                    newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr() - step, -sourceVp.width() / 2.0);
                    // 1 new1 new1 new2 new2 2
                    CheckIfCollisionUsingLine(point1, newpoint1, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    CheckIfCollisionUsingLine(newpoint1, newpoint2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    CheckIfCollisionUsingLine(newpoint2, point2, &isocc, resolution / 2.0);
                    if (isocc)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    expandLength[i] += step;
                    if (expandLength[i] >= limitBound)
                    {
                        NotFinishTable[i] = 0.0;
                        break;
                    }
                    NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(-step / 2.0, 0.0);
                    NewsourceVp.set_length(NewsourceVp.length() + step);
                    sourcePt = NewsourcePt;
                    sourceVp = NewsourceVp;
                    break;
                }
            }
        }

        Eigen::Vector2d point1, norm1;
        // LF
        point1 = rawPt + egoR * Eigen::Vector2d(rawVp.length() / 2.0 + rawVp.d_cr() + expandLength[1], rawVp.width() / 2.0 + expandLength[0]);
        norm1 << -sin(yaw), cos(yaw);
        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;
        Eigen::Vector2d point2, norm2;
        // RF
        // point2 = sourcePt+egoR*Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
        point2 = rawPt + egoR * Eigen::Vector2d(rawVp.length() / 2.0 + rawVp.d_cr() + expandLength[1], -rawVp.width() / 2.0 - expandLength[2]);
        norm2 << cos(yaw), sin(yaw);
        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;
        Eigen::Vector2d point3, norm3;
        // RB
        //  point3 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
        point3 = rawPt + egoR * Eigen::Vector2d(-rawVp.length() / 2.0 + rawVp.d_cr() - expandLength[3], -rawVp.width() / 2.0 - expandLength[2]);
        norm3 << sin(yaw), -cos(yaw);
        hPoly.col(2).head<2>() = norm3;
        hPoly.col(2).tail<2>() = point3;
        Eigen::Vector2d point4, norm4;
        // LB
        // point4 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
        point4 = rawPt + egoR * Eigen::Vector2d(-rawVp.length() / 2.0 + rawVp.d_cr() - expandLength[3], rawVp.width() / 2.0 + expandLength[0]);
        norm4 << -cos(yaw), -sin(yaw);
        hPoly.col(3).head<2>() = norm4;
        hPoly.col(3).tail<2>() = point4;
        hPolys_.push_back(hPoly);

        if (index < statelist.size())
        {
#ifdef PRINT
            std::cout << "expandLength_[" << index << "]:" << expandLength[0] << "," << expandLength[1] << "," << expandLength[2] << "," << expandLength[3] << std::endl;
            std::cout << "Matrix " << index << ":" << hPoly << std::endl;

// std::cout<<"rawPt["<<index<<"]:"<<rawPt[0]<<","<<rawPt[1]<<std::endl;
#endif
        }

        index += 1;
    }

    //  //print hPolys_ Matrix
    //  std::cout<<"getrectangleConst->hPolys_"<<std::endl;
    //  std::cout<<"hPolys_.size():"<< hPolys_.size()<<std::endl;
    //   for (size_t i = 0; i < hPolys_.size(); ++i) {
    //     std::cout << "Matrix " << i + 1 << ":" << hPolys_[i] << std::endl;
    //   }
    return hPolys_;
}

bool CheckIfCollisionUsingLine(const Eigen::Vector2d p1,
                               const Eigen::Vector2d p2, bool *res, double checkl)
{
    bool issuccess = false;
    for (double dl = 0.0; dl < (p2 - p1).norm(); dl += checkl)
    {
        Eigen::Vector2d pos = (p2 - p1) * dl / (p2 - p1).norm() + p1;
        issuccess = CheckCollisionUsingGlobalPosition(pos, res);
        // map_->CheckCollisionUsingGlobalPosition(pos,res);
        if (*res)
        { // is collision
            // issuccess=true;
            return issuccess;
        }
    }
    issuccess = CheckCollisionUsingGlobalPosition(p2, res);
    // map_->CheckCollisionUsingGlobalPosition(p2,res);
    return issuccess;
}

bool CheckCollisionUsingGlobalPosition(const Vec2f &p_w, bool *res)
{
    bool issuccess = 0;
    const int N_DIM = 2;
    std::array<int, 2> dims_size_;
    std::array<double, 2> origin_;
    std::array<float, 2> dims_resolution_;
    float gres = pathfind_parameters.XY_GRID_RESOLUTION;
    dims_resolution_ = {gres, gres};
    origin_ = {pathfind_parameters.MAXX, pathfind_parameters.MAXY};
    dims_size_[0] = std::ceil((pathfind_parameters.MAXX - pathfind_parameters.MINX) / gres);
    dims_size_[1] = std::ceil((pathfind_parameters.MAXY - pathfind_parameters.MINY) / gres);
    std::array<int, N_DIM> coord = {};
    std::array<int, N_DIM> cal_mono_coord = {};
    for (int i = 0; i < N_DIM; ++i)
    {
        coord[i] = std::ceil((origin_[i] - p_w[i]) / dims_resolution_[i]);
        if ((coord[i] < 0) || (coord[i] > dims_size_[i]))
        {
            issuccess = true;
            *res = true;
            return issuccess;
        }
    }
    cal_mono_coord = coord;
    int mono_idx = 0;
    if (coord[0] == 0)
    {
        cal_mono_coord[0] = 1;
    }
    if (coord[1] == 0)
    {
        cal_mono_coord[1] = 1;
    }
    mono_idx = (cal_mono_coord[0] - 1) * dims_size_[0] + (cal_mono_coord[1] - 1);

    if ((obstmap[mono_idx].Status == 0) || (obstmap[mono_idx].Status == 3) || (obstmap[mono_idx].Status == 4))
    {
        issuccess = true;
        *res = true;
        return issuccess;
    }
    issuccess = true;
    *res = false;
    return issuccess;
}