/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// 実は、rfだけではなく、別の脚でも同様に接触検出をしたい。コールバック関数は各脚ごとに作成します（つまりは4このコールバック関数）しかし、中身はスッキリさせたい。つまり、ある関数を作成し、その関数で各脚の処理を行う。

// その関数の役割としては、下記です。
// std::pairを用いて、タイムスタンプ（現在時間ではなく、gazeboから取得した値）と接触の有無を示すboolの変数を表現する。
// これまでのやり方で、接触の有無を判断する。
// std::vectorによって、さきほどのstd::pairの可限長配列を表現している。そして、push_backで、最新のstd::pairの値をさきほどのstd::vectorにpush_backする。

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <champ/utils/urdf_loader.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ContactManager.hh"
#include <boost/algorithm/string.hpp>
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <array>

class ContactSensor: public rclcpp::Node
{
  bool foot_contacts_[4];
  std::array<builtin_interfaces::msg::Time, 4> foot_stamp_;  // 各足の接触時刻
  std::vector<std::string> foot_links_;
  rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
  gazebo::transport::SubscriberPtr gazebo_sub;

public:
  ContactSensor()
  : foot_contacts_{false,false,false,false},
    Node("contacts_sensor", rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true))
  {
    std::vector<std::string> joint_names;
    joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());

    // URDF からリンク名を取得して足リンクを登録
    foot_links_.push_back(joint_names[2]);
    foot_links_.push_back(joint_names[6]);
    foot_links_.push_back(joint_names[10]);
    foot_links_.push_back(joint_names[14]);

    contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

    // タイムスタンプ配列の初期化
    for (auto &t : foot_stamp_) {
      t.sec = 0;
      t.nanosec = 0;
    }

    // Gazebo クライアント初期化
    gazebo::client::setup();
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo_sub = node->Subscribe("/rf_foot_contact", &ContactSensor::gazeboCallback_, this);
  }

  // Gazebo 時刻比較用のヘルパ
  static bool newer(const builtin_interfaces::msg::Time& a,
                    const builtin_interfaces::msg::Time& b)
  {
    return (a.sec > b.sec) || (a.sec == b.sec && a.nanosec > b.nanosec);
  }

// 先頭の include 群のままでOK（追加は不要）
// using namespace は使わず、明示的に gazebo::msgs を書いています。

void gazeboCallback_(ConstContactsPtr &_msg)
{
  // // --- 1) 今周期の接触フラグをリセット -----------------------
  // for (size_t i = 0; i < 4; ++i) {
  //   foot_contacts_[i] = false;
  // }

  // const int n = _msg->contact_size();

  // if (n == 0) {
  //   // 接触なし
  //   std::cout << "[/rf_foot_contact] no contact" << std::endl;
  // } else {
  //   // 接触あり
  //   std::cout << "[/rf_foot_contact] contact_size = " << n << std::endl;

  //   for (int i = 0; i < n; ++i) {
  //     const auto &c = _msg->contact(i);

  //     // --- 衝突ペア名 ---
  //     const std::string col1 = c.collision1();
  //     const std::string col2 = c.collision2();
  //     std::cout << "   collisions: [" << col1 << "] <-> [" << col2 << "]\n";

  //     // --- タイムスタンプ ---
  //     const auto &gz_t = c.time();
  //     builtin_interfaces::msg::Time ros_t;
  //     ros_t.sec     = gz_t.sec();
  //     ros_t.nanosec = gz_t.nsec();

  //     // --- 足リンクとの一致をチェックしてフラグ更新 ---
  //     auto linkFromCollision = [](const std::string& coll) -> std::string {
  //       std::vector<std::string> tokens;
  //       boost::split(tokens, coll, [](char ch){ return ch == ':'; });
  //       return (tokens.size() >= 3) ? tokens[2] : std::string();
  //     };
  //     const std::string link1 = linkFromCollision(col1);
  //     const std::string link2 = linkFromCollision(col2);

  //     for (size_t j = 0; j < 4; ++j) {
  //       if ((!link1.empty() && foot_links_[j] == link1) ||
  //           (!link2.empty() && foot_links_[j] == link2)) {
  //         foot_contacts_[j] = true;
  //         foot_stamp_[j]    = ros_t;  // 最新タイムスタンプを保存
  //       }
  //     }
  //   }
  // }

  // --- 2) Foot contact state をROSメッセージで publish -----
  // publishContacts();
}




};

void exitHandler(int sig)
{
  gazebo::client::shutdown();
  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContactSensor>();
  rclcpp::Rate loop_rate(1);
  // rclcpp::spin(node);
  while (rclcpp::ok())
  {
    // node->publishContacts();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}








// UNUSED -----------------------------------------

/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
// #include <iomanip>  // 先頭付近で一度だけインクルードしておくと便利（小数桁表示用）

// #include <rclcpp/rclcpp.hpp>
// #include <iostream>
// #include <champ/utils/urdf_loader.h>
// #include <gazebo/transport/transport.hh>
// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/gazebo_client.hh>
// #include "gazebo/physics/World.hh"
// #include "gazebo/physics/ContactManager.hh"
// #include <boost/algorithm/string.hpp>
// #include <champ_msgs/msg/contacts_stamped.hpp>
// #include <builtin_interfaces/msg/time.hpp>
// #include <array>

// class ContactSensor: public rclcpp::Node
// {
//   bool foot_contacts_[4];
//   std::array<builtin_interfaces::msg::Time, 4> foot_stamp_;  // 各足の接触時刻
//   std::vector<std::string> foot_links_;
//   rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
//   gazebo::transport::SubscriberPtr gazebo_sub;

// public:
//   ContactSensor()
//   : foot_contacts_{false,false,false,false},
//     Node("contacts_sensor", rclcpp::NodeOptions()
//           .allow_undeclared_parameters(true)
//           .automatically_declare_parameters_from_overrides(true))
//   {
//     std::vector<std::string> joint_names;
//     joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());

//     // URDF からリンク名を取得して足リンクを登録
//     foot_links_.push_back(joint_names[2]);
//     foot_links_.push_back(joint_names[6]);
//     foot_links_.push_back(joint_names[10]);
//     foot_links_.push_back(joint_names[14]);

//     contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

//     // タイムスタンプ配列の初期化
//     for (auto &t : foot_stamp_) {
//       t.sec = 0;
//       t.nanosec = 0;
//     }

//     // Gazebo クライアント初期化
//     gazebo::client::setup();
//     gazebo::transport::NodePtr node(new gazebo::transport::Node());
//     node->Init();

//     gazebo_sub = node->Subscribe("/rf_foot_contact", &ContactSensor::gazeboCallback_, this);
//   }

//   // Gazebo 時刻比較用のヘルパ
//   static bool newer(const builtin_interfaces::msg::Time& a,
//                     const builtin_interfaces::msg::Time& b)
//   {
//     return (a.sec > b.sec) || (a.sec == b.sec && a.nanosec > b.nanosec);
//   }

//   void gazeboCallback_(ConstContactsPtr &_msg)
//   {
//     // 今周期の接触フラグをリセット
//     for (size_t i = 0; i < 4; ++i) {
//       foot_contacts_[i] = false;
//     }

//     // すべての contact を走査
//     for (int i = 0; i < _msg->contact_size(); ++i)
//     {
//       std::vector<std::string> results;
//       std::string collision = _msg->contact(i).collision1();

//       std::cout << "## collision name " << i << " = " << collision << std::endl;

//       for (int i = 0; i < _msg->contact_size(); ++i)
//       {
//         const auto &gz_t = _msg->contact(i).time(); // gazebo::msgs::Time
//         std::cout << "[gazeboCallback] contact[" << i << "] gz_time = "
//                   << gz_t.sec() << "s " << gz_t.nsec() << "ns" << std::endl;
//       }

//       boost::split(results, collision, [](char c){return c == ':';});

//       // Gazebo の Contact に入っている time を取得
//       const auto &gz_t = _msg->contact(i).time();  // gazebo::msgs::Time
//       builtin_interfaces::msg::Time ros_t;
//       ros_t.sec     = gz_t.sec();
//       ros_t.nanosec = gz_t.nsec();

//       for (size_t j = 0; j < 4; ++j)
//       {
//         if (foot_links_[j] == results[2])
//         {
//           foot_contacts_[j] = true;

//           // 同一周期内に複数回の接触が来ても最新のものを保持
//           if (newer(ros_t, foot_stamp_[j])) {
//             foot_stamp_[j] = ros_t;
//           }

//           break;
//         }
//       }
//     }
// // for (int i = 0; i < 4; i++)
// // {
// //     std::cout << "foot_stamp_[" << i << "] = "
// //               << foot_stamp_[i].sec << "s "
// //               << foot_stamp_[i].nanosec << "ns"
// //               << std::endl;
// // }
//     publishContacts();
//   }

//   void publishContacts()
//   {
//     champ_msgs::msg::ContactsStamped contacts_msg;

//     // 通常は ROS のクロックを使用（/use_sim_time=true ならシム時刻）
//     // contacts_msg.header.stamp = this->get_clock()->now();
// 	contacts_msg.header.stamp = foot_stamp_[0];

//     contacts_msg.contacts.resize(4);
//     for (size_t i = 0; i < 4; ++i) {
//       contacts_msg.contacts[i] = foot_contacts_[i];
//     }

//     contacts_publisher_->publish(contacts_msg);

//     // デバッグ出力: 足ごとの接触時刻
//     // for (size_t i = 0; i < 4; ++i) {
//     //   if (foot_contacts_[i]) {
//     //     RCLCPP_INFO(this->get_logger(),
//     //       "Foot[%zu] contact TRUE at %d.%09u",
//     //       i, foot_stamp_[i].sec, foot_stamp_[i].nanosec);
//     //   }
//     // }
//   }
// };

// void exitHandler(int sig)
// {
//   gazebo::client::shutdown();
//   rclcpp::shutdown();
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<ContactSensor>();
//   rclcpp::Rate loop_rate(50);
//   rclcpp::spin(node);
//   // while (rclcpp::ok())
//   // {
//   //   node->publishContacts();
//   //   rclcpp::spin_some(node);
//   //   loop_rate.sleep();
//   // }
//   rclcpp::shutdown();
//   return 0;
// }
