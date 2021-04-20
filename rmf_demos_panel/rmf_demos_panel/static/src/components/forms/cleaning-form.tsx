import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Typography from '@material-ui/core/Typography';
import Autocomplete, { AutocompleteRenderInputParams } from '@material-ui/lab/Autocomplete';
import { useFormStyles } from "../styles";
import { NotificationSnackbar, NotificationTypes } from '../fixed-components/notification-snackbar';

interface CleaningFormProps {
  cleaningZones: string[],
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

export const CleaningForm = (props: CleaningFormProps): React.ReactElement => {
  const { cleaningZones, submitRequest, timeAndPriority } = props;
  const { minsFromNow, priority, setTimeError, setMinsFromNow, setPriority, setPriorityError } = timeAndPriority;
  const [allZones, setZones] = React.useState(cleaningZones);
  const [targetZone, setTargetZone] = React.useState('');

  //snackbar
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [messageType, setMessageType] = React.useState(NotificationTypes.Success);

  //errors
  const [zoneError, setZoneError] = React.useState("");
  
  const classes = useFormStyles();
  
  React.useEffect(() => {
    setZones(cleaningZones);
  }, [cleaningZones]);

  const cleanUpForm = () => {
    setMinsFromNow(0);
    setTargetZone("");
    setPriority(0);
    setPriorityError("");
  }

  const isFormValid = () => {
    let isValid = true;
    if(priority < 0 || priority > 1) {
      setPriorityError("Priority can only be 0 or 1");
      isValid = false;
    }
    if(minsFromNow < 0) {
      setTimeError("Start time cannot be negative");
      isValid = false;
    }
    if(targetZone.length === 0) {
      setZoneError("Cleaning zone cannot be an empty field");
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
    setSnackbarMessage(message);
    setMessageType(NotificationTypes.Success);
    setOpenSnackbar(true);
  }

  const createRequest = () => {
    let start_time = minsFromNow;
    let cleaning_zone = targetZone;
    let request = {};
    let priority_option = priority;
    request = { task_type: "Clean",
                start_time: start_time,
                priority: priority_option,
                description: {'cleaning_zone': cleaning_zone} }
    return request;
  }

  const handleSubmit = async (ev: React.FormEvent) => {
    try {
      ev.preventDefault();
      if(isFormValid()) {
        let request = createRequest();
        let response = await submitRequest(request);
        if (response && response["task_id"] === '') {
          showErrorSnackbar(`Cleaning Request failed! ${response["error_msg"]}`);
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
        <Box className={classes.form} role="cleaning-form">
            <div className={classes.divForm}>
            <Typography variant="h6">Schedule a Clean Request</Typography>
                <Autocomplete
                options={allZones}
                getOptionLabel={(zone) => zone}
                id="set-cleaning-zone"
                openOnFocus
                onChange={(_, value) => setTargetZone(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Pick a zone" variant="outlined" margin="normal" helperText={zoneError} error={!!zoneError}/>}
                value={targetZone ? targetZone : null}
                />
            </div>
            <div className={classes.buttonContainer}>
                <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Request</Button>
                 {openSnackbar && <NotificationSnackbar type={messageType} message={snackbarMessage} closeSnackbarCallback={() => setOpenSnackbar(false)} />}
            </div>
        </Box>
    );
} 

export default CleaningForm;