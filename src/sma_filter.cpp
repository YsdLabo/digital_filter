#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// 単純移動平均　Simple Moving Average
class SmaFilter : public rclcpp::Node
{
public:
	// コンストラクタ
	SmaFilter() : Node("sma_filter_node"),
		NUM_SMA_(this->declare_parameter<int>("number", 5)), // パラメータ宣言と取得
		data_(std::make_unique<double[]>(NUM_SMA_))
	{
		// サブスクライバーを作成（/imuトピックを購読）
		sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu> (
			"/imu", 
			rclcpp::SensorDataQoS(),
			std::bind(&SmaFilter::callback_imu, this, std::placeholders::_1)
		);
		// パブリッシャーを作成（/imu_outトピックを配信）
		pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu> (
			"/imu_out",
			rclcpp::SensorDataQoS()
		);
	}
private:
	// 購読用コールバック関数
	void  callback_imu( const  sensor_msgs::msg::Imu::SharedPtr  msg )
	{
		sensor_msgs::msg::Imu imu_raw = *msg;	// 受信メッセージをコピー

		// バッファ内データの更新
		data_[num_] = imu_raw.linear_acceleration.z;    // 要素番号numに加速度zを記録
		num_++;             // 要素番号の更新
		if(num_ == NUM_SMA_) num_ = 0;    // 要素番号がデータ点数と同じになったらリセット

		// 移動平均の計算
		double sma_z = 0.0;
		for(int  i=0; i<NUM_SMA_; i++) {
			sma_z += data_[i];			// NUM_SMA_個のデータを合計
		}
		sma_z /= (double)NUM_SMA_;		// 平均計算

		// 配信
		auto imu_mod = imu_raw;
		imu_mod.linear_acceleration.z = sma_z;
		pub_imu_->publish(imu_mod);

		// 表示
		RCLCPP_INFO( this->get_logger( ), "z: %f  sma_z: %f",
			imu_raw.linear_acceleration.z,
			imu_mod.linear_acceleration.z
		);
	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
	int NUM_SMA_;
	int num_ = 0;
	std::unique_ptr<double[]> data_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SmaFilter>());
	rclcpp::shutdown();
	return 0;
}
