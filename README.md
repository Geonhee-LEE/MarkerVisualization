# MarkerVisualization

## How to use

```cpp

// Visualization
#include "visualization.h"

std::shared_ptr<MarkerVisualization> marker_visualization_;
marker_visualization_ = std::make_shared<MarkerVisualization>(private_nh);

// for loop
    marker_visualization_->deleteAllMarkers();

    geometry_msgs::Point aPoints;
    geometry_msgs::Point bPoints;

    aPoints.x = start_pose.position.x;
    aPoints.y = start_pose.position.y;
    bPoints.x = last_pose.position.x;
    bPoints.y = last_pose.position.y;

    scales scale = XXLARGE;
    marker_visualization_->publishLine(aPoints, bPoints, GREEN, scale);


    std::size_t id = 0;
    geometry_msgs::Pose pose;
    pose.position.x = poses[i].x;
    pose.position.y = poses[i].y;
    pose.orientation = tf::createQuaternionMsgFromYaw(poses[i].theta);
    marker_visualization_->publishArrow(pose, id);
    id++;

    marker_visualization_->trigger();
```
