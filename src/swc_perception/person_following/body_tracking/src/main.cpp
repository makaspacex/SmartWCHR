// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>
#include <memory>
#include <sstream>

#include "include/body_tracking.h"


int main(int argc, char *argv[]) {
  std::stringstream ss;
  ss << "\n\tThis is body tracking package.\n\n"
     << "\nWake up action is putting your left arm up.\n"
     << "Cancel action is putting your right arm up.\n"
     << "============================================\n";
  std::cout << ss.str() << std::endl;

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  // GetNodes包含进来两个节点，一个设置参数的节点，一个发布cmd_vel指令的节点
  auto nodes = TrackingManager::Instance()->GetNodes();
  for (auto &node : nodes) {
    exec.add_node(node);
  }

  /* 
  大概的逻辑：
  每当这个subscriber接收到ai_msg消息，就会把接收到的ai_msg送到TrackingManager的成员对象队列中
  然后会调用TrackingManager类的构造函数
  在构造函数当中，将之前得到的ai_msg出队，并根据ai_msg计算出速度指令并发布
  因此要实现行人重识别，需要增加一个订阅者，订阅发布的原始图片，并与订阅ai_msg的订阅者同步
  队列中的内容可以改成一个unorder_map，存放image-ai_msg对
  然后就根据逻辑进行代码的修改
  */

/*
目前是订阅者smart_msg_subscriber类中包含了一个函数指针，指向回调函数FeedSmart，FeedSmart的作用是把ai_msg传入队列
在smart_msg_subscriber类的构造函数中，定义了一个订阅者，订阅内容为手势识别的消息，在这个订阅者的回调函数中，调用FeedSmart，把smart_msg加入到队列中
为什么要这么做？
因为main函数这里不是直接构造一个订阅者，而是实例化一个类
在实例化MessageSyncNode类的过程中，会调用其构造函数
在构造函数中，从队列中取出消息，完成对速度的计算和发布
*/


// 这一段是之前修改之前的原始跟随项目的代码
/*
  auto smart_msg_subscriber = std::make_shared<SmartMsgSubscriber>(
      "ai_msg_sub_node",
      std::bind(&TrackingManager::FeedSmart,          // FeedSmart函数，将订阅到的ai_msg信息加入TrackingManager的队列中
                TrackingManager::Instance(),          // 单例实例，整个程序只有一个TrackingManager对象
                std::placeholders::_1));

  exec.add_node(smart_msg_subscriber);
*/



/*
  这是之前的逻辑，同步订阅者同时接收/image和SmartMsg，将消息入队进行处理
*/
  auto message_sync_node = std::make_shared<MessageSyncNode>(
    "message_sync_node",
    std::bind(&TrackingManager::FeedSmart2,          // FeedSmart1，FeedSmart的改版，需要传入两个参数而不是一个
              TrackingManager::Instance(),          // 跟之前一样
              std::placeholders::_1,
		          std::placeholders::_2,
              std::placeholders::_3
		)
  );


  exec.add_node(message_sync_node);

  // auto miniPublisher_node = std::make_shared<MinimalPublisher> ();
  // exec.add_node(miniPublisher_node);



  exec.spin();

  // release node before shutdown!
  TrackingManager::Instance()->Release();

  rclcpp::shutdown();

  std::cout << "tracking node exit!\n";



  return 0;
}

