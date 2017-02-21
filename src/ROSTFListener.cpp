/**
 * @file ROSTFListener.cpp
 * @brief 
 *	Implimentation of ROSTFListener
 * @author Iwase
 * @date 2016.9.28.
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include <ros_module/ROSInterface.hpp>

//----------------------------------------------------------
// ROSTFListener
//----------------------------------------------------------
ROSTFListener::ROSTFListener(const std::string& src, const std::string& dst):
	exit(false),
	valid(false),
	initialized(false),
	freq(DEFAULT_FREQUENCY),
	src_name(src),
	dst_name(dst),
	curr_trans{ {0, 0, 0} },
	curr_rot{ {0, 0, 0, 1} }
{};
ROSTFListener::~ROSTFListener()
{
	mtx.lock();
	exit = true;
	mtx.unlock();
	if(listen_thread.joinable()) listen_thread.join();
};
void ROSTFListener::init()
{
	ros::NodeHandle n;
	initialized = true;
	// Start publish loop
	try {
		listen_thread = std::thread( [this]{ listen_loop(); } );
	}
	catch (std::system_error& e) {
		std::cout << e.what() << std::endl;
	}

}
void ROSTFListener::set_source(const std::string& src)
{
	std::lock_guard<std::mutex> lock(mtx);
	src_name = src;
	valid = false;
};
void ROSTFListener::set_destination(const std::string& dst)
{
	std::lock_guard<std::mutex> lock(mtx);
	dst_name = dst;
	valid = false;
};
void ROSTFListener::set_transform(const std::string& src, const std::string& dst)
{
	std::lock_guard<std::mutex> lock(mtx);
	src_name = src;
	dst_name = dst;
	valid = false;
};
bool ROSTFListener::valid_data()
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	return valid;
};
bool ROSTFListener::get_translation(std::array<double, 3>& trans)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	std::copy(std::begin(curr_trans), std::end(curr_trans), std::begin(trans));
	return valid;
};
bool ROSTFListener::get_translation(tf::Vector3& trans)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	trans = tf::Vector3(curr_trans[0], curr_trans[1], curr_trans[2]);
	return valid;
};
bool ROSTFListener::get_rotation(std::array<double, 4>& rot)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	std::copy(std::begin(curr_rot), std::end(curr_rot), std::begin(rot));
	return valid;
};
bool ROSTFListener::get_rotation(tf::Quaternion& rot)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	rot = tf::Quaternion(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]);
	return valid;
};
bool ROSTFListener::get_transform(std::array<double, 3>& trans, std::array<double, 4>& rot)
{	
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	std::copy(std::begin(curr_trans), std::end(curr_trans), std::begin(trans));
	std::copy(std::begin(curr_rot), std::end(curr_rot), std::begin(rot));
	return valid;
};
bool ROSTFListener::get_transform(tf::Vector3& trans, tf::Quaternion& rot)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	trans = tf::Vector3(curr_trans[0], curr_trans[1], curr_trans[2]);
	rot = tf::Quaternion(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]);
	return valid;
};
std::array<double, 3> ROSTFListener::transform(const std::array<double, 3>& pos)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx);
	assert(valid);
	//std::cout << "trans :" << curr_trans[0] << curr_trans[1] << curr_trans[2] << std::endl;
	tf::Transform tr(tf::Quaternion(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]), tf::Vector3(curr_trans[0], curr_trans[1], curr_trans[2]));
	tf::Vector3 r_val = tr(tf::Vector3(pos[0], pos[1], pos[2]));
	return std::array<double, 3>{r_val[0], r_val[1], r_val[2]};
};
tf::Vector3 ROSTFListener::transform(const tf::Vector3& pos)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx);
	assert(valid);
	tf::Transform tr(tf::Quaternion(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]), tf::Vector3(curr_trans[0], curr_trans[1], curr_trans[2]));
	return tr(pos);
};
tf::Quaternion ROSTFListener::transform(const tf::Quaternion& q)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx);
	assert(valid);
	return q * tf::Quaternion(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]);
};
std::array<double, 3> ROSTFListener::inverse_transform(const std::array<double, 3>& pos)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	tf::Quaternion q(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]);
	tf::Transform tr(q, tf::Vector3(curr_trans[0], curr_trans[1], curr_trans[2]));
	tf::Transform tr_inv(tr.inverse());
	tf::Vector3 r_val = tr_inv(tf::Vector3(pos[0], pos[1], pos[2]));
	return std::array<double, 3>{r_val[0], r_val[1], r_val[2]};
};
tf::Vector3 ROSTFListener::inverse_transform(const tf::Vector3& pos)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx); 
	assert(valid);
	tf::Quaternion q(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]);
	tf::Transform tr(q, tf::Vector3(curr_trans[0], curr_trans[1], curr_trans[2]));
	tf::Transform tr_inv(tr.inverse());
	return tr_inv(pos);
};
tf::Quaternion ROSTFListener::inverse_transform(const tf::Quaternion& q)
{
	assert(initialized);
	std::lock_guard<std::mutex> lock(mtx);
	assert(valid);
	return q * tf::Quaternion(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3]).inverse();
};
void ROSTFListener::listen_loop()
{

    tf::TransformListener listener;
    tf::StampedTransform tf;
	ros::Rate loop_timer(freq);
	// publish loop
	bool loop_continue(true);
	while(loop_continue){

		bool try_success(false);
		try{			
            ros::Time now = ros::Time(0);

            listener.waitForTransform(src_name.c_str(), dst_name.c_str(), now, ros::Duration(MAXIMUM_DELAY));
            listener.lookupTransform(src_name.c_str(), dst_name.c_str(), now, tf);

            try_success = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("ROSTFListener - %s",ex.what());
            try_success = false;
        }
        catch ( ... ){
            ROS_ERROR("ROSTFListener - ^^^^^^^^^^^^^^^ UNKNOWN EXCEPTION ^^^^^^^^^^^^^^");
            try_success = false;
        }
		// copy to buffer
		std::array<double, 3> trans_buf = { tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z() };
		std::array<double, 4> rot_buf = { tf.getRotation().x(), tf.getRotation().y(), tf.getRotation().z(), tf.getRotation().w() };

		mtx.lock();

		// update members
		if(try_success){
			valid = try_success;
			std::copy(std::begin(trans_buf), std::end(trans_buf), std::begin(curr_trans));
			std::copy(std::begin(rot_buf), std::end(rot_buf), std::begin(curr_rot));
		}

		loop_continue = !exit;

		mtx.unlock();

		loop_timer.sleep(); //sleep
	}
};

