#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// RCフィルタ
class RcFilter : public rclcpp::Node
{
public:
	// コンストラクタ
	RcFilter() : Node("rc_filter_node"),
		Fc_(this->declare_parameter<double>("fc", 10.0))  // パラメータ宣言と取得
	{
		// サブスクライバーを作成（/imuトピックを購読）
		sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu> (
			"/imu", 
			rclcpp::SensorDataQoS(),
			std::bind(&RcFilter::callback_imu, this, std::placeholders::_1)
		);
		// パブリッシャーを作成（/imu_rcfトピックを配信）
		pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu> (
			"/imu_rcf",
			rclcpp::SensorDataQoS()
		);
	}
private:
	// 購読用コールバック関数
	void  callback_imu( const  sensor_msgs::msg::Imu::SharedPtr  msg )
	{
		sensor_msgs::msg::Imu imu_raw = *msg;	// 受信メッセージをコピー
		rclcpp::Time current_stamp(imu_raw.header.stamp);
		double acc_z = imu_raw.linear_acceleration.z;

		// 指数平均の計算
		if(!first_time_) {
			double dt = (current_stamp - last_stamp_).seconds();
			double alpha = // ここに数式を入れましょう
			rcf_z_ = // ここに数式を入れましょう
		}
		else {
			rcf_z_ = acc_z;
			first_time_ = false;
		}

		// 配信
		auto imu_filtered = imu_raw;
		imu_filtered.linear_acceleration.z = rcf_z_;
		pub_imu_->publish(imu_filtered);

		// 表示
		RCLCPP_INFO( this->get_logger( ), "z: %f  rcf_z: %f",
			imu_raw.linear_acceleration.z,
			imu_filtered.linear_acceleration.z
		);
		
		last_stamp_ = current_stamp;
	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
	double Fc_;
	double rcf_z_ = 0.0;
	rclcpp::Time last_stamp_;
	bool first_time_ = true;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RcFilter>());
	rclcpp::shutdown();
	return 0;
}
