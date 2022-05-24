#include <rclcpp/rclcpp.hpp>

template <class TeleopType>
class   TestableTeleop : public TeleopType
{
  public:

   TestableTeleop(const rclcpp::NodeOptions & options):
     TeleopType(options)
   {

   }

   std::shared_ptr<rclcpp::Node> get_node() const
   {
     return TeleopType::node_;
   }


   std::map<std::string,int> get_mapping() const
   {
     return TeleopType::joy_->get_mapping();
   }

};



