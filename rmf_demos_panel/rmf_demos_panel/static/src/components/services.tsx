const API_SERVER_ADD = "http://" + location.hostname + ":8080"
const GUI_SERVER_ADD = "http://" + location.host

//API endpoints
export const getRobots = async () => {
    try {
        let response = await fetch(API_SERVER_ADD + '/robot_list');
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
        let response = await fetch(API_SERVER_ADD + '/task_list');
        if(response) {
            let data = await response.json()
            console.log("Populate tasks: ", data);
            return data;
        }
    } catch (err) {
        throw new Error(err.message);
    }
}

export const submitRequest = async (request: {}) => {
    try {
        let response = await fetch(API_SERVER_ADD + '/submit_task', {
            method: "POST",
            body: JSON.stringify(request),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .then(res => res.json());
        return response;
      } catch (err) {
        return err;
      }
}

export const cancelTask = async (id: string) => {
    try {
        let response = await fetch(API_SERVER_ADD + '/cancel_task', {
            method: "POST",
            body: JSON.stringify({ task_id: id }),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .then(res => res.json())
        return response;
    } catch (err) {
        return err;
    }
}

export const submitAllTasks = (taskList: any[]) => {
    let i = 0;
    let timerIdList = taskList.map((task) => {
        return setTimeout(async () => {
            await submitRequest(task);
        }, 900*(++i));
    });
    return timerIdList;
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
