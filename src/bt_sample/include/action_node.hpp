#pragma once
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "ros_node.hpp"

using namespace BT;

// Treeノードの定義
namespace MyActionNodes{
    // 1. Counterノード: 入力された文字列の文字数を数える
    class Counter : public SyncActionNode {
    public:
        Counter(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){ }

        // 必須：Portの情報を記述する
        static PortsList providedPorts() {
            return { OutputPort<int>("out_num"),
                     InputPort<std::string>("in_text") }; // 識別用に名前をつける
        }
        
        // ノードが呼び出されると一度だけ実行される
        NodeStatus tick() override {
            std::cout << "call Counter" << std::endl;
            
            // InputPortの値を受け取る
            Expected<std::string> msg = getInput<std::string>("in_text");
            if (!msg) { // Inputの値が適切でないときの処理
                throw BT::RuntimeError("missing required input [in_text]: ", msg.error() );
            }
            std::string in_text = msg.value();
            int out_num = in_text.length();
            
            // OutputPortに値をセット
            setOutput("out_num", out_num);
            
            return NodeStatus::SUCCESS;
        }
    };

    // 2. Displayノード: 入力された文字列の文字数を表示する
    class Display : public SyncActionNode {
    public:
        Display(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){ }

        // Portの情報を記述する
        static PortsList providedPorts() {
            return { InputPort<int>("in_num") };
        }
        
        // ノードが呼び出されると一度だけ実行される
        NodeStatus tick() override {
            std::cout << "call Display" << std::endl;
            
            // InputPortの値を受け取る
            Expected<int> msg = getInput<int>("in_num");
            if (!msg) { // Inputの値が適切でないときの処理
                throw BT::RuntimeError("missing required input [in_num]: ", msg.error() );
            }
            int in_num = msg.value();

            std::cout << "入力された文字列の文字数は " << in_num << " です" << std::endl;
            
            return NodeStatus::SUCCESS;
        }
    };

    // 3. GetTextノード: サーバーから文字列を取得する
    class GetText : public StatefulActionNode {
    public:
        GetText(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        // Portの情報を記述する
        static PortsList providedPorts() {
            return { OutputPort<std::string>("out_text") };
        }

        // ノードが呼び出されると一度だけ実行される
        NodeStatus onStart() override {
            std::cout << "call GetText" << std::endl;
            return NodeStatus::RUNNING;
        }
        
        // Running状態のときに実行される
        NodeStatus onRunning() override {

            // ros_node.hppのget_text()関数から文字列を取得
            std::string out_text = ros_node->get_text();
            
            // OutputPortに値をセット
            setOutput("out_text", out_text);
            
            return NodeStatus::SUCCESS;
        }

        void onHalted() override {
            std::cout << "interrupt GetText" << std::endl;
        }
    };
}