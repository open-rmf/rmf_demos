import React from "react";
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Typography from '@material-ui/core/Typography';
import { WorldContext } from '../fixed-components/app-context';
import { showErrorMessage } from "../fixed-components/messages";
import { useFormStyles } from "../styles";

interface TaskRequest {
  task_type: string,
  start_time: number,
  description: any
}

interface ScheduledTaskFormProps {
  submitTaskList: (taskList: any[]) => void;
}

const ScheduledTaskForm = (props: ScheduledTaskFormProps): React.ReactElement => {
  const { config } = React.useContext(WorldContext);
  const { submitTaskList } = props;
  const [taskList, setTaskList] = React.useState<string | ArrayBuffer>('');
  const [deliveryOptions, setDeliveryOptions] = React.useState({});
  const placeholder = `eg. [
{"task_type":"Loop", "start_time":0, "description": {"num_loops":5, "start_name":"coe", "finish_name":"lounge"}},
{"task_type":"Delivery", "start_time":0, "description": {"option": "coke"}},
{"task_type":"Loop", "start_time":0, "description": {"num_loops":5, "start_name":"cubicle_2", "finish_name":"supplies"}}
]`

  React.useEffect(() => {
    if(Object.keys(config).length > 0) {
      setDeliveryOptions(config.task.Delivery.option);
    } else {
      setDeliveryOptions({});
    }
  }, [config]);
  
  const createTaskDescription = (deliveryTask: string): {} => {
    let newDesc = deliveryOptions[deliveryTask];
    return newDesc;
  }

  const convertTaskList = () => {
    let globalTaskList = [];
    let tempTaskList: string | any = taskList;
    let jsonTaskList = JSON.parse(tempTaskList);
    globalTaskList = jsonTaskList.map((task: TaskRequest) => {
      if(task.task_type == "Delivery") {
        let taskDesc = createTaskDescription(task.description.option);
        return {
          task_type: task.task_type,
          start_time: task.start_time,
          description: taskDesc
        }
      } else {
        return task
      }
    });
    return globalTaskList;
  }
  
  const isFormValid = () => {
    if(taskList === "" || taskList === `[]` || taskList === `{}`) {
      showErrorMessage("Unable to submit an empty task list");
      return false;
    }
    return true;
  }
  
  const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      let globalList = convertTaskList();
      submitTaskList(globalList);
      setTaskList('');
    }
  }
  
  const readTaskFile = (e: React.ChangeEvent<HTMLInputElement>) => {
    let file = e.target.files[0];
    const reader = new FileReader();
    reader.onload = function(event) {
      let result = event.target.result;
      setTaskList(result);
    };
    reader.readAsText(file);
  }
  
  const classes = useFormStyles();
  
  return (
    <Box className={classes.form} role="scheduled-task-form">
        <div className={classes.divForm}>
          <Typography variant="h6">Submit a List of Tasks</Typography>
          </div>
        <div className={classes.buttonContainer}>
          <Button variant="contained" color="primary" className={classes.button}>
            <input type="file" id="task_file" name="task_file" onChange={readTaskFile} />
          </Button>
        </div>
        <div className={classes.divForm}>
          <TextField
                multiline
                rows={5}
                placeholder={placeholder}
                variant="outlined"
                fullWidth
                value={taskList || ''}
                onChange={(e) => setTaskList(e.target.value)}
              />
        </div>
        <div className={classes.buttonContainer}>
          <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Task List</Button>
        </div>
    </Box>
  )
}

export default ScheduledTaskForm;