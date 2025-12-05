# interactive_marker_pose_server

Provides one or multiple 6-DoF interactive markers in RViz and publishes their poses as `geometry_msgs/PoseStamped`.
Useful as a general-purpose “pose handle” (goal markers, waypoints, tool targets, etc.).

## Launch

```bash
ros2 launch interactive_marker_pose_server interactive_marker_pose_server.launch.yaml
```

This loads the config file:

```
config/interactive_marker_pose_server.yaml
```

---

## Configuration

Single marker (publishes on `pose`):

```yaml
interactive_marker_pose_server:
  ros__parameters:
    frame_id: "map"
    marker_name: "interactive_marker_pose_server"
    init_pose: [0, 0, 0, 0, 0, 0]
```

Multiple markers (each publishes on `<marker>/pose`):

```yaml
interactive_marker_pose_server:
  ros__parameters:
    frame_id: "map"
    marker_names: ["goal", "start"]
    init_pose: [0, 0, 0, 0, 0, 0]
```

Optional parameters:

* `init_pose_topic` — PoseStamped topic used to set the initial pose (TF-transformed into `frame_id`).
* `reset_topic` — Send an empty message to reset all markers.

---

## RViz

1. Add **InteractiveMarkers** display.
2. Set server name: `interactive_marker_pose_server`.
3. Move the markers to update the published pose topics.
