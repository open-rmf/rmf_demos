## RMF Demos Panel
Here we describe additional details of `rmf_demos_panel`. This package uses a simple web server
to expose the essential RMF topics/services to users as web endpoints. Currently, this lite implementation is 
useful for task submission and observing status of on-going tasks and robots in RMF.

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
ros2 launch rmf_demos office.launch.xml
```
Launch the dashboard
```
firefox localhost:5000
```

> Note that this will download the latest webpack "GUI" hosted on `rmf_demos` github page. Thus internet is
required when you are running the `gui_server` for the first time.

---

## Development Mode (local npm compilation)

Currently the webpack bundle is located here `https://open-rmf.github.io/rmf_demos/VERSION/app.bundle.js`. Thus this is useful
for personnel who would like to compile a local react webpack bundle (for testing). **Else you can skip this portion.**

### Dependencies: 
 - npm (node version > 12, see: [node](https://nodejs.org/en/download/package-manager/))

### Compilation
```bash
cd ~/rmf_ws

# change the npm prefix according to the path to "rmf_demos_panel/static/"
npm install --prefix src/demonstrations/rmf_demos/rmf_demos_panel/rmf_demos_panel/static
npm run build --prefix src/demonstrations/rmf_demos/rmf_demos_panel/rmf_demos_panel/static

colcon build --packages-select rmf_demos_panel
```

### API Endpoints

Endpoints | Type | Parameters | Description
--- | --- | --- | ---
/submit_task | POST | task description json | This supports submission of task. This response with an assigned task_id is the task is accepted
/cancel_task | POST | task_id string | Cancel an existing task in rmf.
/task_list | GET | NA | Get list of task status in RMF (include active and terminated tasks)
/robot_list | GET | NA | Get list of Robot states in RMF
/task_status | SocketIO | NA | Constant broadcast of task list
/robot_states | SocketIO | NA | Constant broadcast of robot list

#### Simple CURL Test

```bash
# Submit Task (POST)
curl -X POST http://localhost:8080/submit_task \
-d '{"task_type":"Loop", "start_time":0, "description": {"start_name": "coe", "finish_name": "pantry", "num_loops":1}}' \
-H "Content-Type: application/json" 

# Get Task List (GET)
curl http://localhost:8080/task_list
```

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

## Create your own GUI

There are 2 web-based server running behind the scene, namely:

1. `gui_server` (port `5000`): Providing the static gui to the web client. Non RMF dependent
2. `api_server` (port `8080`): Hosting all endpoints for gui clients to interact with RMF

To create your own custom GUI, you will only require to replace the current `gui_server` package with your 
own custom implementation, . The `api_server` remains the same.

---

## Note
- Edit the `dashboard_config.json` to configure the input of the Demo World GUI Task Submission.
The dashboard config file is located here: `rmf_demos_dashboard_resources/$WORLD/dashboard_config.json`.
- server ip is configurable via `WEB_SERVER_IP_ADDRESS` in the `dashboard.launch.xml`
- The `api_server` outputs and stores a summarized log: `web_server.log`.
- cancel task will not be working. A fully functional cancel will be introduced in a future PR.
- Rosdep will automatically install system version of `python3-flask` and `python3-flask-cors`. Yet we will download `flask-socketio` (5.x) separately via pip since the ubutuntu packaged version is too old.
- To purely run GUI server (without ros for testing), run :
  ```bash
  export DASHBOARD_CONFIG_PATH=src/rmf/rmf_demos/rmf_demos_dashboard_resources/office/dashboard_config.json
  python src/rmf/rmf_demos/rmf_demos_panel/rmf_demos_panel/gui_server.py
  # Then, access localhost:5000 on browser
  ```
