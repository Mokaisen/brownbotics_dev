#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/String"

class BrownbotIKSeq : public rclcpp::Node{
public:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_seq_; 
    rclcpp::Subscriber<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::TimerBase::SharedPtr timer_;  

    std::vector<float> joint_states_now_;
    std::vector<float> joint_states_prev_;
    std::vector<std::string> seq_moves_; 

    BrownbotIKSeq() : Node("BrownbotIKSeq"){
        pub_seq_ = this->create_publisher<std_msgs::msg::String>("trigger_goal",10);
        timer_ = this->create_wall_timer(1.0, std::bind(&BrownbotIKSeq::timer_callback, this));

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states",10,
            std::bind(&BrownbotIKSeq::topic_callback),this,_1);
        
        // Load sequence of movements
        seq_moves_ = {
            "0,-0.69,0.2,0,-180,90",
            "0,-0.69,0.06,0,-180,90",
            "0,-0.69,0.06,0,-180,90,0.33",
            "0,-0.69,0.2,0,-180,90,0.33",
            "-0.6,0.0,0.06,0,-180,90,0.33",
            "0,-0.69,0.2,0,-180,90,0.33",
            "0,-0.69,0.06,0,-180,90,0.33",
            "0,-0.69,0.06,0,-180,90,0.0",
            "0,-0.69,0.2,0,-180,90,0.0"
        }
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        joint_states_now_ = msg->position;
    }   

    void timer_callback(){
        
        float dot_product = 1.0;

        if(joint_states_prev.empty()){
            joint_states_prev_ = joint_states_now_;
        }else{
            dot_product = std::inner_product(joint_states_now_.begin(),joint_states_now_.end(),
                                             joint_states_prev_.end(), 0.0);
        }
        
        std::string seq_step = "";
        if(dot_product != 1.0 && seq_moves_.size()>0){
            seq_step = seq_moves_.front();

            //publish to the controller
            auto msg_seq = std_msgs::msg::String;
            msg_seq.data = seq_step;
            pub_seq_->publish(msg_seq); 
        }

    }

};

int main(int agrc, char ** argv){
    printf("send sequence of target positions to brownbot");


}