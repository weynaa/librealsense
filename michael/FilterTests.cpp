#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <pcl/pclHeader.h>
#include <pcl/io/pcd_io.h>


struct RSPipe {
	rs2::pipeline pipe;
	rs2::config config;

	RSPipe(rs2::pipeline&& p, rs2::config&& cfg) : pipe(p), config(cfg) {

	}
	RSPipe(const RSPipe& copy) = default;
	void operator=(RSPipe&& p) {
		this->pipe = std::move(p.pipe);
		this->config = std::move(p.config);
	}
};

Eigen::Matrix4f intrinsicsToMatrix(const rs2_intrinsics& intrinsic) {
	assert(intrinsic.model == RS2_DISTORTION_NONE);
	Eigen::Matrix4f intMat = Eigen::Matrix4f::Identity();
	intMat(0, 0) = intrinsic.fx;
	intMat(1, 1) = intrinsic.fy;
	intMat(0, 2) = intrinsic.ppx;
	intMat(1, 2) = intrinsic.ppy;
	return intMat;
}

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int main(int argc, char** argv) {
	std::vector<RSPipe> pipes;
	{
		rs2::config config;
		config.enable_device_from_file("../artekmed_data/test_recording02/rscapture1.bag");
		auto pipe = rs2::pipeline();
		pipes.emplace_back(std::move(pipe), std::move(config));
	}
	{
		rs2::config config;
		config.enable_device_from_file("../artekmed_data/test_recording02/rscapture2.bag");
		auto pipe = rs2::pipeline();
		pipes.emplace_back(std::move(pipe), std::move(config));
	}
	for (auto& p : pipes) {
		p.pipe.start(p.config);
	}

	std::vector<rs2::frame> new_frames;
	for (auto&& pipe : pipes)
	{
		rs2::frameset fs = pipe.pipe.wait_for_frames();
		new_frames.emplace_back(fs.get_depth_frame());
	}
	//Referenzcamera für extrinsics
	auto base_cam = pipes.front().pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH);
	std::vector<Eigen::Matrix4f> intrinsics;
	std::vector<rs2_extrinsics> extrinsics; // Extrinsics don't work in recovery mode (from file)

	auto points = boost::make_shared < pcl::PointCloud<pcl::PointXYZ>>();
	points->width = 1;
	points->height = 0;
	for (const auto& frame : new_frames) {
		auto intrinsic = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
		auto t = intrinsicsToMatrix(intrinsic);
		intrinsics.push_back(t);
		t=t.inverse().eval();
		auto numPoints = intrinsic.width * intrinsic.height;
		points->points.reserve(numPoints);

		points->height += numPoints;
		for (int y = 0; y < intrinsic.height; ++y) {
			for (int x = 0; x < intrinsic.width; ++x) {
				//Kann auch einfach rs2_deproject_pixel_to_point gemacht werden
				auto vec = Eigen::Vector4f{(float)x,(float)y,1,1};
				auto depth = (float)((uint16_t*)frame.get_data())[y * intrinsic.width + x];
				vec = t * vec;
				vec.x() *= depth;
				vec.y() *= depth;
				vec.z() *= depth;
				points->points.emplace_back(vec.x(), vec.y(), vec.z());
			}
		}
		if (&frame == &new_frames[0]) {
			pcl::io::savePCDFile("rs2Data_1.pcd", *points, true);
		}
	}

	//pcl::io::savePCDFile("rs2Data.pcd", *points, true);

	return 0;
}