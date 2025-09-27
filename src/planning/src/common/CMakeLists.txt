cmake_minimum_required(VERSION 3.22)
project(common)

# 设置依赖
set(CONFIG_DEPENDS
  "rclcpp"
  "yaml-cpp"
  "ament_index_cpp"
  )

set(MATH_DEPENDS
  "rclcpp"
  "base_msgs"
  "geometry_msgs"
  "nav_msgs"
  )

# 添加动态库

# 安装动态奎、节点和相关目录


