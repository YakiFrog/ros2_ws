#pragma once
#include "rclcpp/rclcpp.hpp"
#include "bt_sample/srv/call_text.hpp"

using namespace std::chrono_literals;

// BTNodeクラスの定義(rosのノード)
/*
ここのプログラムは、ROS2に関するもの。BTとは関係ない。が、BTのノードで使用する関数もある。
*/

class BTNode : public rclcpp::Node{
    public:
    // bt_node: ROS2のサービス通信を行う
    BTNode() : Node("bt_node"){
        std::cout << "bt_node is called" << std::endl;
        // クライアントの作成(bt_sample/srv/CallText)
        call_text_cli = create_client<bt_sample::srv::CallText>("call_text");
        // サービスの作成
        while(!call_text_cli->wait_for_service(1s)){ // サービスが利用可能になるまで待機
            if(!rclcpp::ok()){ // ノードが終了している場合
                break;
            }
            std::cout << "call_text service not available" << std::endl; // サービスが利用可能になるまでのメッセージ
        }
        std::cout << "call_text service available" << std::endl; // サービスが利用可能になったメッセージ
    }

    // action_node.hppのGetTextノードで使用
    std::string get_text(){
        // サービスを呼び出す(bt_sample/srv/CallText)
        auto request = std::make_shared<bt_sample::srv::CallText::Request>();
        // サービスの結果を取得
        auto future_result = call_text_cli->async_send_request(request);
        // サービスの結果を取得できるまで待機
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS){
            return future_result.get()->text; // サービスの結果を返す
        }
        // サービスの結果を取得できなかった場合
        std::cout << "can't get future_result" << std::endl;
        return ""; // 空文字を返す
    }

    private:
    rclcpp::Client<bt_sample::srv::CallText>::SharedPtr call_text_cli; // クライアントの宣言
};

std::shared_ptr<BTNode> ros_node; // BTNodeクラスのインスタンスを保持する変数