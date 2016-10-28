/**
 * @file ROSFramework.hpp
 * @brief Framework for using ros
 * @author Iwase
 * @date 2016.9.26.
 */
#ifndef ROS_FRAMEWORK_HPP
#define ROS_FRAMEWORK_HPP
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <ros/ros.h>
#include <mutex>
#include <thread>
/**
* @class ROSSubscriberInterface
* @brief Template classes for subscription of topics given in the automatic.
* @tparam T Type of message that subscribed from ros
*/
template<class T>
struct ROSSubscriberInterface{
protected:
	static constexpr int DEF_SUBSCRIBER_BUFFER_NUM_ = 32;

protected:
	const std::string topic_name_;
	const int bufffer_num_;

protected:
	bool initialized_;
	bool exit_;
	bool new_data_;

	T val_;

	ros::Subscriber subscriber_;
	std::mutex mtx_;
	std::thread subscribe_thread_;

private:
	void sub_callback(const T& msg)
	{
		std::lock_guard<std::mutex> lock(mtx_);
		val_ = msg;
		new_data_ = true;
	};
	void subscribe_loop()
	{
		bool loop_continue(true);
		// subscribe loop
		while(loop_continue){
			// If Call Available run call back method.
			ros::spinOnce();

			mtx_.lock();
			loop_continue = !exit_;
			mtx_.unlock();
		}
	};

public:
	ROSSubscriberInterface();
	ROSSubscriberInterface(const ROSSubscriberInterface&);
	/**
	 * @brief constructor from topic name.
	 * @param name topic name
	 */
	ROSSubscriberInterface(const std::string& name) : 
		topic_name_(name), 
		bufffer_num_(DEF_SUBSCRIBER_BUFFER_NUM_),
		initialized_(false),
		exit_(false),
		new_data_(false)
	{};
	/**
	 * @brief constructor from topic name and buffer size.
	 * @param name topic name
	 * @param n buffer size in subscriber 
	 */
	ROSSubscriberInterface(const std::string& name, const int n) : 
		topic_name_(name),
		bufffer_num_(n),
		initialized_(false),
		exit_(false),
		new_data_(false)
	{};
	virtual ~ROSSubscriberInterface()
	{
		// exit subscribe loop
		mtx_.lock();
		exit_ = true;
		mtx_.unlock();

		// joint thread
		if(subscribe_thread_.joinable()) subscribe_thread_.join();
	};
	/**
	 * @brief Initialize node and Start the subscribe thread
	 */
	void init()
	{
		assert(!initialized_);
		ros::NodeHandle n;
		// joint state subscriber
		subscriber_ = n.subscribe(topic_name_, bufffer_num_, &ROSSubscriberInterface<T>::sub_callback, this);
		// Start subscribe loop
		try {
			subscribe_thread_ = std::thread( [this]{ ROSSubscriberInterface<T>::subscribe_loop(); } );
		}
		catch (std::system_error& e) {
			std::cout << e.what() << std::endl;
		}
		// initialized
		initialized_ = true;
	}
	/**
	 * @brief getter
	 * @param val 
	 */
	void get(T& val)
	{
		assert(initialized_);
		std::lock_guard<std::mutex> lock(mtx_);
		val = val_;
		new_data_ = false;
	};
	/**
	 * @brief Whether it is updated with new data.
	 * @return New data came : true 
	 */
	bool new_data() const
	{
		return new_data_;
	};
};

/**
* @class ROSServiceInterface
* @brief Template classes for using ROS service.
* @tparam T Type of message used in service
*/
template<class T>
struct ROSServiceInterface{
protected:
	const std::string service_name_;

protected:
	bool initialized_;

	ros::ServiceClient service_client_;

public:		
	ROSServiceInterface();
	ROSServiceInterface(const ROSServiceInterface&);
	/**
	 * @brief constructor from service name.
	 * @param name service name
	 */
	ROSServiceInterface(const std::string& name) : 
		service_name_(name), 
		initialized_(false)
	{};
	virtual ~ROSServiceInterface(){};
	/**
	 * @brief Initialize node and make service client
	 */
	void init()
	{
		ros::NodeHandle n;
	    service_client_ = n.serviceClient<T>(service_name_.c_str());
	    // initialized
	    initialized_ = true;
	};
	/**
	 * @brief Initialize node and make service client
	 * @param msg message.
	 * @return call success : true, failed : false
	 */
	bool call(T& msg)
	{
		assert(initialized_);
		if (!service_client_.call(msg)){
		    ROS_ERROR("ROSServiceInterface - Service call got error!");
		    return false;
		}
		return true;
	};
};

#endif