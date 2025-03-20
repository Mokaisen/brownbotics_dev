#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class BrownbotIKSeq : public rclcpp::Node{
public:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_seq_; 
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_; 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::TimerBase::SharedPtr timer_;  

    std::vector<double> joint_states_now_;
    std::vector<double> joint_states_prev_;
    std::vector<std::string> seq_moves_;

    
    BrownbotIKSeq() : Node("BrownbotIKSeq"){
        pub_seq_ = this->create_publisher<std_msgs::msg::String>("trigger_goal",10);
        pub_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command",10);
        
        timer_ = this->create_wall_timer(5s, std::bind(&BrownbotIKSeq::timer_callback, this));

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states",10,
            std::bind(&BrownbotIKSeq::topic_callback,this,std::placeholders::_1));
        
        // Load sequence of movements
        seq_moves_ = {
            "0,-0.69,0.2,0,-180,90",
            "0,-0.69,0.06,0,-180,90",
            "close_gripper",
            "0,-0.69,0.2,0,-180,90",
            "-0.6,0.0,0.06,0,-180,90",
            "0,-0.69,0.2,0,-180,90",
            "0,-0.69,0.06,0,-180,90",
            "open_gripper",
            "0,-0.69,0.2,0,-180,90"
        };
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        joint_states_now_ = msg->position;
        joint_states_now_.assign(msg->position.begin(), msg->position.begin() + 7);

    }   

    void timer_callback(){
        
        float dot_product = 1.0;

        if(joint_states_prev_.empty()){
            joint_states_prev_ = joint_states_now_;
        }else{
            // Compute norms
            float norm_A = std::sqrt(std::inner_product(joint_states_now_.begin(), 
                                                        joint_states_now_.end(), 
                                                        joint_states_now_.begin(), 0.0));
            float norm_B = std::sqrt(std::inner_product(joint_states_prev_.begin(), 
                                                        joint_states_prev_.end(), 
                                                        joint_states_prev_.begin(), 0.0));
            dot_product = std::inner_product(joint_states_now_.begin(),joint_states_now_.end(),
                                             joint_states_prev_.begin(), 0.0) / (norm_A * norm_B);
        }

        //RCLCPP_INFO(this->get_logger(), "joint states now: %s", vector_to_string(joint_states_now_).c_str());
        //RCLCPP_INFO(this->get_logger(), "joint states prev: %s", vector_to_string(joint_states_prev_).c_str());
        
        std::string seq_step = "empty";
        if(dot_product == 1.0 && seq_moves_.size()>0){
            
            seq_step = seq_moves_.front();
            seq_moves_.erase(seq_moves_.begin());
            RCLCPP_INFO(this->get_logger(), "seq_moves: %s", vector_to_string(seq_moves_).c_str());

            if(seq_step == "close_gripper"){
                auto msg_joints = sensor_msgs::msg::JointState();
                msg_joints.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint","finger_joint"};
                msg_joints.position = joint_states_now_;
                msg_joints.position.push_back(0.33);
                pub_joints_->publish(msg_joints);
            }else if(seq_step == "open_gripper" ){
                auto msg_joints = sensor_msgs::msg::JointState();
                msg_joints.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint","finger_joint"};
                msg_joints.position = joint_states_now_;
                msg_joints.position.push_back(0.0);
                pub_joints_->publish(msg_joints);
            }else{
                //publish to the controller
                auto msg_seq = std_msgs::msg::String();
                msg_seq.data = seq_step;
                pub_seq_->publish(msg_seq);
            } 
        }
        
        RCLCPP_INFO(this->get_logger(), "seq_msg %s", seq_step.c_str());
        RCLCPP_INFO(this->get_logger(), "dot product %f", dot_product);

        joint_states_prev_ = joint_states_now_;
    }
private:    
    template <typename T>
    std::string vector_to_string(const std::vector<T>& vec) {
        std::ostringstream oss;
        for (size_t i = 0; i < vec.size(); ++i) {
            oss << vec[i];
            if (i < vec.size() - 1) oss << ", "; // Add comma separator
        }
        return "[" + oss.str() + "]";
    }

};

int main(int argc, char ** argv){
    (void) argc;
    (void) argv;

    printf("send sequence of target positions to brownbot");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrownbotIKSeq>());
    rclcpp::shutdown();
    return 0;
}