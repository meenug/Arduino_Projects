Custom message structures for NeoPix and Relay board

####Add message header related dependencies in your CMakeLists.txt and package.xml####

#### CMakeLists.txt####

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ...
  message_generation
)
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
  
add_message_files(
   FILES
   ....
   NeoPixel.msg
   relay_channel.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

#### package.xml####
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
