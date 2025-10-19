#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// 指数移動平均　Exponential Moving Average
class EmaFilter : public rclcpp::Node
{
public:
	// コンストラクタ
	EmaFilter() : Node("ema_filter_node"),
		ALPHA_(this->declare_parameter<double>("alpha", 0.1))  // パラメータ宣言と取得
	{
		// サブスクライバーを作成（/imuトピックを購読）
		sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu> (
			"/imu", 
			rclcpp::SensorDataQoS(),
			std::bind(&EmaFilter::callback_imu, this, std::placeholders::_1)
		);
		// パブリッシャーを作成（/imu_emaトピックを配信）
		pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu> (
			"/imu_ema",
			rclcpp::SensorDataQoS()
		);
	}
private:
	// 購読用コールバック関数
	void  callback_imu( const  sensor_msgs::msg::Imu::SharedPtr  msg )
	{
		sensor_msgs::msg::Imu imu_raw = *msg;	// 受信メッセージをコピー

		// 指数平均の計算
		double acc_z = imu_raw.linear_acceleration.z;
		ema_z = // ここに数式を入れましょう

		// 配信
		auto imu_filtered = imu_raw;
		imu_filtered.linear_acceleration.z = ema_z;
		pub_imu_->publish(imu_filtered);

		// 表示
		RCLCPP_INFO( this->get_logger( ), "z: %f  ema_z: %f",
			imu_raw.linear_acceleration.z,
			imu_filtered.linear_acceleration.z
		);
	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
	double ALPHA_;
	double ema_z = 0.0;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EmaFilter>());
	rclcpp::shutdown();
	return 0;
}
