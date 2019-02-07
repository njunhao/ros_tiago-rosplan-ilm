FROM njunhao/ros_tiago-rosplan

COPY /rosplan_interface_ext_action /workspace/ROS/ROSPlan/src/rosplan/rosplan_interface_ext_action

COPY /rosplan_ilm /workspace/ROS/ROSPlan/src/rosplan/rosplan_ilm

COPY /rosplan_ilm_msgs /workspace/ROS/ROSPlan/src/rosplan/rosplan_ilm_msgs

COPY /rosplan_interface_tiago /workspace/ROS/ROSPlan/src/rosplan/rosplan_interface_tiago

COPY /rosplan_interface_mapping/src/RPRoadmapServer.cpp /workspace/ROS/ROSPlan/src/rosplan/rosplan_interface_mapping/src/RPRoadmapServer.cpp

COPY /rosplan_interface_mapping/src/RPSimpleMapServer.cpp /workspace/ROS/ROSPlan/src/rosplan/rosplan_interface_mapping/src/RPSimpleMapServer.cpp

# install package
RUN apt-get install -y ros-kinetic-cv-bridge

# build packages
RUN /bin/bash -c "cd /workspace/ROS/ROSPlan \
	&& rosdep update \
	&& source devel/setup.bash \
	&& catkin build rosplan_interface_ext_action rosplan_ilm_msgs rosplan_ilm rosplan_interface_tiago rosplan_interface_mapping"

# source ROS environment
RUN echo "source /workspace/ROS/ROSPlan/devel/setup.bash --extend" >> /root/.bashrc

# setup symlinks
RUN /bin/bash -c "ln -s /workspace/ROS/ROSPlan/src/rosplan/rosplan_interface_tiago/src/rosplan_interface_grasp/pick_and_place_server2.py /workspace/ROS/tiago_ws/src/tiago_tutorials/tiago_pick_demo/scripts/pick_and_place_server2.py \
    && ln -s /workspace/ROS/ROSPlan/src/rosplan/rosplan_interface_tiago/src/rosplan_interface_grasp/pick_client2.py /workspace/ROS/tiago_ws/src/tiago_tutorials/tiago_pick_demo/scripts/pick_client2.py \
    && ln -s /workspace/ROS/ROSPlan/src/rosplan/rosplan_interface_tiago/worlds/demo_office.world /workspace/ROS/tiago_ws/src/pal_gazebo_worlds/worlds/demo_office.world"