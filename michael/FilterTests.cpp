#include <librealsense2/rs.hpp>
#include <pcl/pclHeader.h>
#include <pcl/io/pcd_io.h>


struct RSPipe {
	rs2::pipeline pipe;
	rs2::config config;

	RSPipe(rs2::pipeline&& p, rs2::config&& cfg) : pipe(p),config(cfg){

	}
	RSPipe(const RSPipe& copy) = default;
	void operator=(RSPipe&& p) {
		this->pipe = std::move(p.pipe);
		this->config = std::move(p.config);
	}
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int main(int argc, char** argv) {
	std::vector<RSPipe> pipes;
	{
		rs2::config config;
		config.enable_device_from_file("G:/artekmed_data/test_recording02/rscapture1.bag");
		auto pipe = rs2::pipeline();
		pipes.emplace_back(std::move(pipe), std::move(config));
	}
	{
		rs2::config config;
		config.enable_device_from_file("G:/artekmed_data/test_recording02/rscapture2.bag");
		auto pipe = rs2::pipeline();
		pipes.emplace_back(std::move(pipe), std::move(config) );
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
	std::vector<rs2_intrinsics> intrinsics;
	std::vector<rs2_extrinsics> extrinsics; // Extrinsics don't work in recovery mode (from file)

	auto points = boost::make_shared < pcl::PointCloud<pcl::PointXYZ>>();
	for (const auto& frame : new_frames) {
		auto intrinsic = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
		intrinsics.emplace_back(intrinsic);
	
		rs2::pointcloud pc;
		auto rs2Points = pc.calculate(frame);
		points->points.reserve(rs2Points.size());
		auto it = rs2Points.get_vertices();
			for (int i = 0; i < rs2Points.size(); ++i) {
				points->points.emplace_back( it->x,it->y,it->z );
				++it;
			}
			if (&frame == &new_frames[0]) {
				pcl::io::savePCDFile("rs2Data_1.pcd", *points, true);
			}
	}

	pcl::io::savePCDFile("rs2Data.pcd", *points,true);

	return 0;
}