import { showErrorMessage, showSuccessMessage } from './fixed-components/messages';

const API_SERVER_ADD = "http://" + location.hostname + ":8080"
const GUI_SERVER_ADD = "http://" + location.hostname + ":5000"

//API endpoints
export const getRobots = async () => {
    try {
        let response = await fetch(API_SERVER_ADD + '/get_robots');
        if(response) {
            let data = await response.json()
            console.log("Populate robots: ", data);
            return data;
        } else {
            return;
        }
    } catch (err) {
        throw new Error(err.message);
    }
}

export const getTasks = async () => {
    try {
        let response = await fetch(API_SERVER_ADD + '/get_task');
        if(response) {
            let data = await response.json()
            console.log("Populate tasks: ", data);
            return data;
        }
    } catch (err) {
        throw new Error(err.message);
    }
}

export const submitRequest = (request: {}, type: string) => {
    try {
        fetch(API_SERVER_ADD + '/submit_task', {
            method: "POST",
            body: JSON.stringify(request),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .then(res => res.json())
        .then(data => {
            JSON.stringify(data);
            let task_id = data["task_id"];
            if (task_id === "")
                showErrorMessage(`${type} Request Failed!`);
            else
                showSuccessMessage(
                    `${type} Request submitted successfully! Task ID: [${task_id}]`);
        });
      } catch (err) {
        console.log(err);
        showErrorMessage(`Unable to submit ${type} Request`);
      }
}

export const cancelTask = (id: string) => {
    try {
        fetch(API_SERVER_ADD + '/cancel_task', {
            method: "POST",
            body: JSON.stringify({ task_id: id }),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .then(res => res.json())
        .then(data => {
            JSON.stringify(data);
            let is_success = data["success"];
            if (is_success)
                showSuccessMessage(`Task ${id} has been cancelled`);
            else
                showErrorMessage(`Unable to cancel Task ${id}`);
        });
    } catch (err) {
        console.log(err);
        showErrorMessage(`Unable to cancel Task ${id}`);
    }
}

export const submitTaskList = (taskList: any[]) => {
    //simulate submission of tasks at intervals
    let res = "Task List submitted successfully";
    let i = 0;
    taskList.forEach((task) => {
      setTimeout(function(i) {
        try {
          fetch(API_SERVER_ADD + '/submit_task', {
            method: "POST",
            body: JSON.stringify(task),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
          })
          .then(res => res.json())
          .then(data => JSON.stringify(data));
        } catch (err) {
          res = "ERROR! " + err;
          showErrorMessage(res);
          console.log('Unable to submit task request');
        }
      }, 900*(++i));
    });
    showSuccessMessage(res);
}

export const getDashboardConfig = async () => {
    try {
        let response = await fetch(GUI_SERVER_ADD + '/dashboard_config');
        if(response) {
            let data = await response.json()
            console.log("Get Dashboard Config", data);
            return data;
        }
    } catch (err) {
        throw new Error(err.message);
    }
}
