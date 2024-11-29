#include "rclcpp/rclcpp.hpp"
#include "bt_sample/srv/call_text.hpp"

using namespace std::chrono_literals;

// TextServerクラスの定義

class TextServer : public rclcpp::Node{
    public:
    TextServer() : Node("text_server"){
        auto service_callback = [this](const std::shared_ptr<bt_sample::srv::CallText::Request> request, std::shared_ptr<bt_sample::srv::CallText::Response> response) -> void{
            std::cout << "input text : ";
            std::cin >> input_text;
            response->text = input_text;
        };
        srv = create_service<bt_sample::srv::CallText>("call_text", service_callback);
    }
    private:
    std::string input_text;
    rclcpp::Service<bt_sample::srv::CallText>::SharedPtr srv;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TextServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}