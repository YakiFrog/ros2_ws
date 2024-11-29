#include "../include/action_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h" // For Groot2Publisher

using namespace MyActionNodes; // action_node.hppで定義した名前空間を使用
using namespace BT; // BehaviorTreeの名前空間を使用

// メイン関数
int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  ros_node = std::make_shared<BTNode>(); // BTNodeクラスのインスタンスを作成
  BT::BehaviorTreeFactory factory; // ノードの登録を行う
  factory.registerNodeType<Counter>("Counter"); // Counterノードを登録
  factory.registerNodeType<Display>("Display"); // Displayノードを登録
  factory.registerNodeType<GetText>("GetText"); // GetTextノードを登録
  // XMLファイルからBehaviorTreeを作成
  std::string package_path = ament_index_cpp::get_package_share_directory("bt_sample");
  factory.registerBehaviorTreeFromFile(package_path + "/config/main_bt.xml");
  BT::Tree tree = factory.createTree("MainBT");
  printTreeRecursively(tree.rootNode());

  // BehaviorTreeの実行(RUNNINGにする)
  NodeStatus status = NodeStatus::RUNNING;

  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  // ここで、ノードが実行されている
  while(status == NodeStatus::RUNNING && rclcpp::ok()){
    rclcpp::spin_some(ros_node);
    status = tree.tickOnce(); // statusにノードの状態を格納(RUNNING, SUCCESS, FAILURE, IDLE)
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1秒待機
  }

  rclcpp::shutdown();
  return 0;
}