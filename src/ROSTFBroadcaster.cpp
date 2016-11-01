/**
 * @file ROSTFBroadcaster.cpp
 * @brief 
 *	Implimentation of ROSTFBroadcaster
 * @author Iwase
 * @date 2016.9.28.
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "ROSTFBroadcaster.hpp"

//----------------------------------------------------------
// ROSTFBroadcaster
//----------------------------------------------------------
 ROSTFBroadcaster::ROSTFBroadcaster():
	exit_(false),
	freq_(DEFAULT_FREQUENCY),
	src_name_("No_Name_Src"),
	dst_name_("No_Name_Dst"),
	trans_{ {0, 0, 0} },
	rot_{ {0, 0, 0, 1} }
{};
ROSTFBroadcaster::ROSTFBroadcaster(const std::string& src_name, const std::string& dst_name):
	exit_(false),
	freq_(DEFAULT_FREQUENCY),
	src_name_(src_name),
	dst_name_(dst_name),
	trans_{ {0, 0, 0} },
	rot_{ {0, 0, 0, 1} }
{};
ROSTFBroadcaster::ROSTFBroadcaster(const std::string& src_name, const std::string& dst_name, const std::array<double, 3>& t, const std::array<double, 4>& r):
	exit_(false),
	freq_(DEFAULT_FREQUENCY),
	src_name_(src_name),
	dst_name_(dst_name),
	trans_(t),
	rot_(r)
{};
ROSTFBroadcaster::~ROSTFBroadcaster()
{
	mtx_.lock();
	exit_ = true;
	mtx_.unlock();
	if(broadcast_thread_.joinable()) broadcast_thread_.join();
};
void ROSTFBroadcaster::init()
{
	ros::NodeHandle n;
	// Start publish loop
	try {
		broadcast_thread_ = std::thread( [this]{ broadcast_loop(); } );
	}
	catch (std::system_error& e) {
		std::cout << e.what() << std::endl;
	}
}
void ROSTFBroadcaster::set_name(const std::string& src_name, const std::string& dst_name)
{
	std::lock_guard<std::mutex> lock(mtx_); 
	src_name_ = src_name;
	src_name_ = dst_name;
};
void ROSTFBroadcaster::set_translation(const std::array<double, 3>& trans)
{
	std::lock_guard<std::mutex> lock(mtx_); 
	std::copy(std::begin(trans), std::end(trans), std::begin(trans_));
};
void ROSTFBroadcaster::set_rotation(const std::array<double, 4>& rot)
{
	std::lock_guard<std::mutex> lock(mtx_); 
	std::copy(std::begin(rot), std::end(rot), std::begin(rot_));
};
bool ROSTFBroadcaster::set_transform(const std::array<double, 3>& trans, const std::array<double, 4>& rot)
{
	std::lock_guard<std::mutex> lock(mtx_); 
	std::copy(std::begin(trans), std::end(trans), std::begin(trans_));
	std::copy(std::begin(rot), std::end(rot), std::begin(rot_));
};
void ROSTFBroadcaster::set_transform(const std::string& src_name, const std::string& dst_name, const std::array<double, 3>& trans, const std::array<double, 4>& rot)
{
	std::lock_guard<std::mutex> lock(mtx_); 
	src_name_ = src_name;
	src_name_ = dst_name;
	std::copy(std::begin(trans), std::end(trans), std::begin(trans_));
	std::copy(std::begin(rot), std::end(rot), std::begin(rot_));
};
void ROSTFBroadcaster::broadcast_loop()
{

    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    std::string src_name;
    std::string dst_name;
	ros::Rate loop_timer(freq_);

	// publish loop
	bool loop_continue(true);
	while(loop_continue){
		// update members
		mtx_.lock();

		src_name = src_name_;
		dst_name = dst_name_;
		transform.setOrigin( tf::Vector3(trans_[0], trans_[1], trans_[2]) );
    	transform.setRotation( tf::Quaternion(rot_[0], rot_[1], rot_[2], rot_[3]) );
		loop_continue = !exit_;

		mtx_.unlock();

		// send transform
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), src_name.c_str(), dst_name.c_str()));
		
		loop_timer.sleep(); //sleep

	}
};

