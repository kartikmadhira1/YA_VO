#include "../include/Optimizer.hpp"




Optimizer::Optimizer() {
    // TODO Auto-generated constructor stub

}   

Optimizer::convertToEigen(const vector<cv::Point3i> &points3D, const vector<cv::Point2i> &points2D, 
                            VecVector2d &points2Deigen, VecVector3d &points3Deigen) {
    for (int i = 0; i < points3D.size(); i++) {
        Eigen::Vector3d point3d;
        point3d << points3D[i].x, points3D[i].y, points3D[i].z;
        points3Deigen.push_back(point3d);
        Eigen::Vector2d point2d;
        point2d << points2D[i].x, points2D[i].y;
        points2Deigen.push_back(point2d);
    }
}

Optimizer::partialBA(const vector<cv::Point3i> &points3D, const vector<cv::Point2i> &points2D, const Mat &K, Sophus::SE3d &pose) {
    
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // Construct optimzation algorithm
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // Add camera vertex
    VertexPose *vertexPose = new VertexPose();
    vertexPose->setId(0);
    vertexPose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertexPose);

    // Camera parameters Eigen
    Eigen::Matrix3d KEigen;
    KEigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
              K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
              K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);


    // Convert 3d and 2d points in the form of Eigen
    VecVector2d points2Deigen;
    VecVector3d points3Deigen;
    convertToEigen(points3D, points2D, points2Deigen, points3Deigen);


    // Add 3D points and edges
    int index = 1;
    for (size_t i = 0; i < points2Deigen.size(); ++i) {
        auto p2d = points2Deigen[i];
        auto p3d = points3Deigen[i];
        EdgeProjection *edge = new EdgeProjection(p3d, KEigen);
        edge->setId(index);
        edge->setVertex(0, vertexPose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> timeUsed = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << timeUsed.count() << " seconds." << endl;
    cout << "pose estimated by g2o =\n" << vertexPose->estimate().matrix() << endl;
    pose = vertexPose->estimate();
}