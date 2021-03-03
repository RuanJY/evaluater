#pragma once
#ifndef COMMON_H_
#define COMMON_H_
//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <map>

typedef Eigen::Matrix4d Transf;
typedef Eigen::Matrix<double, 3, 1> Point;
typedef Eigen::Matrix<double, 6, 1> State;//x y z roll pitch yaw, angle in rad

class PointMatrix{
    //3d point container
public:
    double stamp = -1;//step
    int num_point = 0;
    const int resize_step = 256;
    const int init_size = 256;//512
    Eigen::Matrix<double, 3, Eigen::Dynamic> point;
    Eigen::Matrix<double, 1, Eigen::Dynamic> delta;
    Point gravity;
    Eigen::MatrixXd V, D;//D is the eigen value, V is vector
    Eigen::MatrixXd min_vector;
    double min_value = -1;
    double delta_sum = -1;
    std::map<double, Eigen::Matrix<double, 3, 1>> eig_sorted;

    PointMatrix(){
        delta = Eigen::MatrixXd::Zero(1, init_size);
        point = Eigen::MatrixXd::Zero(3, init_size);
        delta.fill(100);
    }
    explicit PointMatrix(int size){
        delta = Eigen::MatrixXd::Zero(1, size);
        point = Eigen::MatrixXd::Zero(3, size);
        delta.fill(100);
    }
    explicit PointMatrix(double _stamp){
        delta = Eigen::MatrixXd::Zero(1, init_size);
        point = Eigen::MatrixXd::Zero(3, init_size);
        stamp = _stamp;
        delta.fill(100);
    }
    PointMatrix&  clear(){
        num_point = 0;
        point.fill(0);
        delta.fill(100);
        stamp = -1; gravity.fill(0); V.fill(0); D.fill(0); min_vector.fill(0);
    }
    PointMatrix&  clear_quick(){
        num_point = 0;
        //point.fill(0);
        //delta.fill(100);
        stamp = -1; gravity.fill(0); V.fill(0); D.fill(0); min_vector.fill(0);
    }
    PointMatrix(const PointMatrix& pointM_copy){//copy function

        if(pointM_copy.num_point > point.cols()){
            point.resize(3,pointM_copy.num_point);
            delta.resize(1,pointM_copy.num_point);
        }
        clear();
        stamp = pointM_copy.stamp;
        gravity = pointM_copy.gravity;
        V = pointM_copy.V;
        D = pointM_copy.D;
        min_vector = pointM_copy.min_vector;
        min_value = pointM_copy.min_value;
        point.leftCols(pointM_copy.num_point) = pointM_copy.point.leftCols(pointM_copy.num_point);
        delta.leftCols(pointM_copy.num_point) = pointM_copy.delta.leftCols(pointM_copy.num_point);
        num_point = pointM_copy.num_point;
    }
    PointMatrix& operator = (const PointMatrix & pointM_equal){

        if(pointM_equal.num_point > point.cols()){
            point.resize(3,pointM_equal.num_point);
            delta.resize(1,pointM_equal.num_point);
        }
        clear();
        stamp = pointM_equal.stamp;
        gravity = pointM_equal.gravity;
        V = pointM_equal.V;
        D = pointM_equal.D;
        min_vector = pointM_equal.min_vector;
        min_value = pointM_equal.min_value;
        point.leftCols(pointM_equal.num_point) = pointM_equal.point.leftCols(pointM_equal.num_point);
        delta.leftCols(pointM_equal.num_point) = pointM_equal.delta.leftCols(pointM_equal.num_point);
        num_point = pointM_equal.num_point;
        return *this;
    }
    PointMatrix& operator = (const Eigen::MatrixXd & point_equal){

        if(point_equal.cols() > point.cols()){
            point.resize(3,point_equal.cols());
            delta.resize(1,point_equal.cols());
        }
        clear();
        point.leftCols(point_equal.cols()) = point_equal.topRows(3);
        num_point = point_equal.cols();
        return *this;
    }
    PointMatrix& operator += (const PointMatrix & pointM_add){

        if(pointM_add.num_point == 0){
            return *this;
        }
        int space_left = int(point.cols()) - (pointM_add.num_point + num_point);
        if(space_left < -1*resize_step){
            point.conservativeResize(Eigen::NoChange_t(3), pointM_add.num_point+num_point+resize_step);
            delta.conservativeResize(Eigen::NoChange_t(1), pointM_add.num_point+num_point+resize_step);
        }
        else if(space_left < 0){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            delta.conservativeResize(Eigen::NoChange_t(1), point.cols()+resize_step);
            point.rightCols(resize_step).fill(0);
            delta.rightCols(resize_step).fill(100);
        }
        point.middleCols(num_point, pointM_add.num_point) = pointM_add.point.leftCols(pointM_add.num_point);
        delta.middleCols(num_point, pointM_add.num_point) = pointM_add.delta.leftCols(pointM_add.num_point);
        num_point += pointM_add.num_point;
        return *this;
    }
    /*PointMatrix& operator += (const Eigen::Matrix<double, 3, 1> & point_add){

        if( num_point+1 > point.cols()){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            delta.conservativeResize(Eigen::NoChange_t(1), point.cols()+resize_step);
            point.rightCols(resize_step).fill(0);
            delta.rightCols(resize_step).fill(100);
        }
        point.col(num_point) = point_add;
        num_point++;
        return *this;
    }
    PointMatrix& operator += (double point_add_delta){
        delta(0,num_point) = point_add_delta;
    }*/
    void addPoint(const Eigen::Matrix<double, 3, 1> & point_add){
        if( num_point+1 > point.cols()){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            delta.conservativeResize(Eigen::NoChange_t(1), point.cols()+resize_step);
            point.rightCols(resize_step).fill(0);
            delta.rightCols(resize_step).fill(100);
        }
        point.col(num_point) = point_add;
        delta(0,num_point) = 100;
        num_point++;
    }
    void addPointWithDelta(const Eigen::Matrix<double, 4, 1> & point_add_with_delta){
        if( num_point+1 > point.cols()){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            delta.conservativeResize(Eigen::NoChange_t(1), point.cols()+resize_step);
            point.rightCols(resize_step).fill(0);
            delta.rightCols(resize_step).fill(100);
        }
        point.col(num_point) = point_add_with_delta.topRows(3);
        delta(0,num_point)   = point_add_with_delta(3,0);
        num_point++;
    }
    Eigen::Matrix<double, 4, 1> getPointWithDelta(int index){
        Eigen::Matrix<double, 4, 1> result_point;
        result_point.topRows(3) = point.col(index);
        result_point(3,0) = delta(0,index);
        return result_point;
    }
/*    PointMatrix& operator += (const Eigen::Matrix<double, 4, 1> & point_add){

        if( num_point+1 > point.cols()){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            delta.conservativeResize(Eigen::NoChange_t(1), point.cols()+resize_step);
            point.rightCols(resize_step).fill(0);
            delta.rightCols(resize_step).fill(100);
        }
        point.col(num_point) = point_add.topRows(3);
        delta.col(num_point) = point_add.row(3);
        num_point++;
        return *this;
    }*/
    void print(){

        std::cout << "stamp: " << stamp  << " num_point: " << num_point << " xyz: " <<  std::endl;
        std::cout << point.leftCols(num_point) <<  std::endl;
        if(delta(0,0)!= 100){
            std::cout << "delta: "<< std::endl << delta.leftCols(num_point) << std::endl;
        }
        std::cout<<"size of matrix: \n"<<point.cols()<<std::endl;
    }
    PointMatrix copyTrans(Transf& _trans){

        PointMatrix _scanturn(*this);//copy, so we can return a new PointMatrix
        Eigen::Matrix<double, 4, Eigen::Dynamic> _point_expend;
        _point_expend = Eigen::MatrixXd::Zero(4,1).replicate(1, num_point);
        _point_expend.topRows(3) = _scanturn.point.leftCols(num_point);
        _point_expend.bottomRows(1).fill(1);
        _point_expend = _trans*_point_expend;
        _scanturn.point.leftCols(num_point) = _point_expend.topRows(3);
        return _scanturn;
    }
    void trans(Transf& _trans){
        if(_trans(0,0) == 0 && _trans(1,0) == 0 && _trans(2,0) == 0){
            std::cout<<"ERROR: Wrong Trans!";
        }

        Eigen::Matrix<double, 4, Eigen::Dynamic> _point_expend;
        _point_expend = Eigen::MatrixXd::Zero(4, num_point);
        _point_expend.topRows(3) = point.leftCols(num_point);
        _point_expend.bottomRows(1).fill(1);
        _point_expend = _trans*_point_expend;
        point.leftCols(num_point) = _point_expend.topRows(3);
    }
    void mse_eig(){

        gravity = point.leftCols(num_point).rowwise().sum()/num_point;

        Eigen::Matrix<double, 3, Eigen::Dynamic> _meshg =
                point.leftCols(num_point) - gravity.replicate(1, num_point);
        Eigen::Matrix3d C = _meshg*_meshg.adjoint()/num_point;
        Eigen::EigenSolver<Eigen::Matrix3d> eig(C);
        D = eig.pseudoEigenvalueMatrix();
        V = eig.pseudoEigenvectors();

        for(int i=0; i<3; i++){
            eig_sorted.emplace(D(i, i), V.col(i));
        }
        for(auto &i_map : eig_sorted){
            //std::cout<<"eig result: "<<i_map.first<<std::endl;//<<i_map.second<<std::endl;
        }
        min_value  = (D(0,0)<D(1,1))?((D(0,0)<D(2,2))?(D(0,0)):(D(2,2))):((D(1,1)<D(2,2))?(D(1,1)):(D(2,2)));//mad!!!
        //std::cout<<"min_value old: "<<min_value<<std::endl;
        min_vector = (D(0,0)<D(1,1))?((D(0,0)<D(2,2))?(V.col(0)):(V.col(2))):((D(1,1)<D(2,2))?(V.col(1)):(V.col(2)));
    }
};

#endif


