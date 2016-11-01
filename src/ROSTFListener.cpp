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
#include "ROSTFListener.hpp"

//----------------------------------------------------------
// ROSTFListener
//----------------------------------------------------------
ROSTFListener::ROSTFListener(const std::string& src, const std::string& dst):
	exit(false),
	valid(false),
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
};
void ROSTFListener::set_destination(const std::string& dst)
{
	std::lock_guard<std::mutex> lock(mtx);
	dst_name = dst;
};
void ROSTFListener::set_transform(const std::string& src, const std::string& dst)
{
	std::lock_guard<std::mutex> lock(mtx);
	src_name = src;
	dst_name = dst;
};
bool ROSTFListener::get_transform(std::array<double, 3>& trans, std::array<double, 4>& rot)
{
	std::lock_guard<std::mutex> lock(mtx); 
	std::copy(std::begin(curr_trans), std::end(curr_trans), std::begin(trans));
	std::copy(std::begin(curr_rot), std::end(curr_rot), std::begin(rot));
	return valid;
};
void ROSTFListener::listen_loop()
{

    tf::TransformListener listener;
    tf::StampedTransform tf;
	ros::Rate loop_timer(freq);
	// publish loop
	bool loop_continue(true);
	while(loop_continue){

		bool try_success;
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

		// copy to buffer
		std::array<double, 3> trans_buf = { tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z() };
		std::array<double, 4> rot_buf = { tf.getRotation().x(), tf.getRotation().y(), tf.getRotation().z(), tf.getRotation().w() };

		mtx.lock();

		// update members
		valid = try_success;
		std::copy(std::begin(trans_buf), std::end(trans_buf), std::begin(curr_trans));
		std::copy(std::begin(rot_buf), std::end(rot_buf), std::begin(curr_rot));

		loop_continue = !exit;

		mtx.unlock();

		loop_timer.sleep(); //sleep
	}
};

