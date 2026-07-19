#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

struct BoxQ
{
	Eigen::Vector3f bboxTransform{};
	Eigen::Quaternionf bboxQuaternion{};
	float cube_length{0.0f};
    float cube_width{0.0f};
    float cube_height{0.0f};
};
struct Box
{
	float x_min{0.0f};
	float y_min{0.0f};
	float z_min{0.0f};
	float x_max{0.0f};
	float y_max{0.0f};
	float z_max{0.0f};
};
#endif