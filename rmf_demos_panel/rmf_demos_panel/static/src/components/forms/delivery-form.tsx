import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Typography from '@material-ui/core/Typography';
import Autocomplete, { AutocompleteRenderInputParams } from '@material-ui/lab/Autocomplete';
import { useFormStyles } from '../styles';
import { NotificationSnackbar, NotificationTypes } from '../fixed-components/notification-snackbar';

interface DeliveryFormProps {
  deliveryOptions: {}
  submitRequest: (request: {}) => Promise<any>;
  timeAndPriority: {
    minsFromNow: number,
    priority: number,
    setTimeError: React.Dispatch<React.SetStateAction<string>>,
    setMinsFromNow: React.Dispatch<React.SetStateAction<number>>,
    setPriority: React.Dispatch<React.SetStateAction<number>>,
    setPriorityError: React.Dispatch<React.SetStateAction<string>>
  }
}

const DeliveryForm = (props: DeliveryFormProps): React.ReactElement => {
  const { deliveryOptions, submitRequest, timeAndPriority } = props;
  const { minsFromNow, priority, setTimeError, setMinsFromNow, setPriority, setPriorityError } = timeAndPriority;
  const classes = useFormStyles();
  const [deliveryTask, setDeliveryTask] = React.useState("");
  const [deliveryOptionKeys, setDeliveryOptionKeys] = React.useState([]);

  //snackbar
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [messageType, setMessageType] = React.useState(NotificationTypes.Success);

  //errors
  const [taskError, setTaskError] = React.useState("");

  React.useEffect(() => {
    let optionKeys = [];
    for (const key in deliveryOptions) {
        optionKeys.push(key);
    }
    setDeliveryOptionKeys(optionKeys);
  }, [deliveryOptions]);

  const isFormValid = () => {
    let isValid = true;
    if(priority < 0 || priority > 1) {
      setPriorityError("Priority can only be 0 or 1");
      isValid = false;
    }
    if(deliveryTask === "") {
      setTaskError("Please select a delivery task");
      isValid = false;
    }
    if(minsFromNow < 0) {
      setTimeError("Start time cannot be negative");
      isValid = false;
    }
    return isValid;
  }

  const showErrorSnackbar = (message: string): void => {
    setSnackbarMessage(message);
    setMessageType(NotificationTypes.Error);
    setOpenSnackbar(true);
  }

  const showSuccessSnackbar = (message: string): void => {
    setSnackbarMessage (message);
    setMessageType(NotificationTypes.Success);
    setOpenSnackbar(true);
  }

  const cleanUpForm = () => {
    setDeliveryTask("");
    setTaskError("");
    setTimeError("");
    setMinsFromNow(0);
    setPriority(0);
    setPriorityError("");
  }
  
  const createTaskDescription = (deliveryTask: string): {} => {
    let newDelivery = deliveryOptions[deliveryTask];
    return newDelivery;
  }

  const createRequest = () => {
    let start_time = minsFromNow;
    let description = createTaskDescription(deliveryTask);
    let request = {};
    let priority_option = priority;
    request = { task_type: "Delivery",
                start_time: start_time,
                priority: priority_option,
                description: description }
    return request;
  }
  
  const handleSubmit = async (ev: React.FormEvent) => {
    try {
      ev.preventDefault();
      if(isFormValid()) {
        let request = createRequest();
        let response = await submitRequest(request);
        if (response && response["task_id"] === '') {
          showErrorSnackbar(`Delivery Request failed! ${response["error_msg"]}`);
        } else {
          showSuccessSnackbar(`Request submitted successfully! Task ID: [${response["task_id"]}]`)
        }
        cleanUpForm();
      }
    } catch(err) {
      return;
    }
  }

  return (
    <Box className={classes.form} role="delivery-form">
      <div className={classes.divForm}>
        <Typography variant="h6">Schedule a Delivery Request</Typography>
        <Autocomplete
          options={deliveryOptionKeys}
          getOptionLabel={(option: any) => option}
          id="set-delivery-task"
          openOnFocus
          onChange={(_, value) => setDeliveryTask(value)}
          renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select delivery task" variant="outlined" margin="normal" error={!!taskError} helperText={taskError} />}
          value={deliveryTask ? deliveryTask : null}
        />
      </div>
      <div className={classes.buttonContainer}>
        <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Request</Button>
        {openSnackbar && <NotificationSnackbar type={messageType} message={snackbarMessage} closeSnackbarCallback={() => setOpenSnackbar(false)} />}
      </div>
    </Box>
  )
}

export default DeliveryForm;
