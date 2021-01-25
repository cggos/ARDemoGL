set(camera_models_root ${PROJECT_SOURCE_DIR}/3rdparties/camera_models)

set(camera_models_src
    ${camera_models_root}/src/chessboard/Chessboard.cc
    ${camera_models_root}/src/calib/CameraCalibration.cc
    ${camera_models_root}/src/camera_models/Camera.cc
    ${camera_models_root}/src/camera_models/CameraFactory.cc
    ${camera_models_root}/src/camera_models/CostFunctionFactory.cc
    ${camera_models_root}/src/camera_models/PinholeCamera.cc
    ${camera_models_root}/src/camera_models/PinholeFullCamera.cc
    ${camera_models_root}/src/camera_models/CataCamera.cc
    ${camera_models_root}/src/camera_models/EquidistantCamera.cc
    ${camera_models_root}/src/camera_models/ScaramuzzaCamera.cc
    ${camera_models_root}/src/sparse_graph/Transform.cc
    ${camera_models_root}/src/gpl/gpl.cc
    ${camera_models_root}/src/gpl/EigenQuaternionParameterization.cc
)
set(camera_models_libs ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
include_directories(${camera_models_root}/include)