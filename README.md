# Brachiograph Simulation 

### Testing controllers

```bash
# Give a controller name like joint1_position_controller, joint2_position_controller, joint3_position_controller.

# "1.0" is the angle value in radians.

rostopic pub /brachiograph/joint1_position_controller/command std_msgs/Float64 "1.0"
```
