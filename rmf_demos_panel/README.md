## RMF Demos Panel
Here we describe additional details of `rmf_demos_panel`. This package uses a simple web server
to expose the essential RMF topics/services to users as web endpoints. Currently, this lite implementation is
useful for task submission and observing status of on-going tasks and robots in RMF.

**For a full-proof web application of RMF, please refer to [rmf-web](https://github.com/open-rmf/rmf-web).**

> Note: not to be confused with the `simple-api-server` in rmf_demos and "prod version" of `api-server` in `rmf-web`

### RMF Dependencies
 - `rmf_task_ros2`
 - `rmf_fleet_msgs`
 - `rmf_demos_dashboard_resources`

### Setup `rmf_demos_panel`

```bash
python3 -m pip install flask-socketio
colcon build --packages-select rmf_demos_panel
```

## Run
Test Run with office world

1. Start Office World
```bash
ros2 launch rmf_demos_gz_classic office.launch.xml
```

#### Simple CURL Test

```bash
# Submit Task (POST)
curl -X POST http://localhost:8083/submit_task \
-d '{"task_type":"Loop", "start_time":0, "description": {"start_name": "coe", "finish_name": "pantry", "num_loops":1}}' \
-H "Content-Type: application/json"

# Get Task List (GET)
curl http://localhost:8083/task_list
```


#### rmf-panel-js

A front end dashboard, [rmf-panel-js](https://github.com/open-rmf/rmf-panel-js) is also provided.
This will will be the client for the rmf_demos's api server.

Launch the a web dashboard: https://open-rmf.github.io/rmf-panel-js/

---

### API Endpoints

Endpoints | Type | Parameters | Description
--- | --- | --- | ---
/submit_task | POST | task description json | This supports submission of task. This response with an assigned task_id is the task is accepted
/cancel_task | POST | task_id string | Cancel an existing task in rmf.
/task_list | GET | NA | Get list of task status in RMF (include active and terminated tasks)
/robot_list | GET | NA | Get list of Robot states in RMF
/task_status | SocketIO | NA | Constant broadcast of task list
/robot_states | SocketIO | NA | Constant broadcast of robot list


---

## Run Sample Tasks

**Submit a list of tasks***
On the right side column of the panel, users are able to select a file which consists of a list of
tasks. Example. for office world, load `rmf_demos_tasks/rmf_demos_tasks/office_tasks.json`.
Once the tasks are populated in the box, hit submit!

More details on the format for the `.json` file is presented below.

For loop requests:
```json
{"task_type":"Loop", "start_time":0, "priority":0, "description": {"num_loops":5, "start_name":"coe", "finish_name":"lounge"}}
```

For delivery requests:
```json
{"task_type":"Delivery", "start_time":0, "priority":0, "description": {"option": "coke"}}
```
Internally, the option `coke` is mapped to a set of parameters required for a delivery request. This mapping can be seen in the `rmf_demos_dashboard_resources/office/dashboard_config.json` file.

For clean requests:
```json
{"task_type":"Clean", "start_time":0, "priority":0, "description":{"cleaning_zone":"zone_2"}}
```

**Submit a task***
User can also submit a single task request via the request form on the top-left side of the page.

The latest robot states and task summaries will be reflected at the bottom portion of the GUI.

---

## Note
- Edit the `dashboard_config.json` to configure the input of the Demo World GUI Task Submission.
The dashboard config file is located here: `rmf_demos_dashboard_resources/$WORLD/dashboard_config.json`.
- server ip is configurable via `RMF_DEMOS_API_SERVER_IP` in the `dashboard.launch.xml`
- The `simple_api_server` outputs and stores a summarized log: `web_server.log`.
- cancel task will not be working. A fully functional cancel will be introduced in a future PR.
- Rosdep will automatically install system version of `python3-flask` and `python3-flask-cors`. Yet we will download `flask-socketio` (5.x) separately via pip since the ubutuntu packaged version is too old.
