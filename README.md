This is an application task that I created for a self-driving car team.
I had zero experience with ROS2, but this application task helped me getting my feet into the water :)

Clone this repo into /src directory in your ros2_wc then build the load from source by running following commands in the /src
  ```
colcon build --packages-select autoronto_app_task_ima
source install/local_setup.bash
  ```

Now you are ready to run the application. It is consisted with 3 components: talker, solver and listener
- talker: implemented in ../src/test_pub.cpp
          every second, generates input to the solver via two topics "input" and "target".
          "input" is an int8multi with payload of integers and "target" is a sum of two integers from the input's payload.
          For testing purpose the payload is a fibonacci sequence 2, 3, ..., 55 shuffled every time.
- solver: implemented in ../src/solver.cpp
          subsribes to topics 'input' and 'target' and finds indices of elements which yields the value of target when summed.
          The solved indices are published as topic 'solution' the message will be type int8multi
- listener: implented in ../src/test_sub.cpp
            simply subscribes to the topic 'solution' and prints the content of message into the log

For demo please watch the video "demo.mov"
