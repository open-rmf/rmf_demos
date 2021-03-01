import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Typography from '@material-ui/core/Typography';
import Autocomplete, { AutocompleteRenderInputParams } from '@material-ui/lab/Autocomplete';
import { useFormStyles } from '../styles';

interface DeliveryFormProps {
  deliveryOptions: {}
  submitRequest: (request: {}, type: string) => void;
  timeAndEvaluator: { minsFromNow: number, evaluator: string, setTimeError: React.Dispatch<React.SetStateAction<string>>, setMinsFromNow: React.Dispatch<React.SetStateAction<number>> }
}

const DeliveryForm = (props: DeliveryFormProps): React.ReactElement => {
  const { deliveryOptions, submitRequest, timeAndEvaluator } = props;
  const { minsFromNow, evaluator, setTimeError, setMinsFromNow } = timeAndEvaluator;
  const classes = useFormStyles();
  const [deliveryTask, setDeliveryTask] = React.useState("");
  const [deliveryOptionKeys, setDeliveryOptionKeys] = React.useState([]);

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
    if(deliveryTask === "") {
      setTaskError("Please select a delivery task");
      return false;
    }
    if(minsFromNow < 0) {
      setTimeError("Start time cannot be negative");
      return false;
    }
    return true;
  }

  const cleanUpForm = () => {
    setDeliveryTask("");
    setTaskError("");
    setTimeError("");
    setMinsFromNow(0);
  }
  
  const createTaskDescription = (deliveryTask: string): {} => {
    let newDelivery = deliveryOptions[deliveryTask];
    return newDelivery;
  }

  const createRequest = () => {
     let start_time = minsFromNow;
      let description = createTaskDescription(deliveryTask);
      let request = {};
      if (evaluator.length > 0 ){
        let evaluator_option = evaluator;
        request = { task_type: "Delivery",
                    start_time: start_time,
                    evaluator: evaluator_option,
                    description: description }
      } else {
        request = { task_type: "Delivery",
                    start_time: start_time,
                    description: description }
      }
      return request;
  }
  
    const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      let request = createRequest();
      submitRequest(request, "Delivery");
      cleanUpForm();
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
      </div>
    </Box>
  )
}

export default DeliveryForm;
