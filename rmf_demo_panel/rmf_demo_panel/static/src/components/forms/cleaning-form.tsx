import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Typography from '@material-ui/core/Typography';
import Autocomplete, { AutocompleteRenderInputParams } from '@material-ui/lab/Autocomplete';
import { useFormStyles } from "../styles";

interface CleaningFormProps {
  cleaningZones: string[],
  submitRequest: (request: {}, type: string) => void;
  timeAndEvaluator: { minsFromNow: number, evaluator: string, setTimeError: React.Dispatch<React.SetStateAction<string>>, setMinsFromNow: React.Dispatch<React.SetStateAction<number>>}
}

export const CleaningForm = (props: CleaningFormProps): React.ReactElement => {
  const { cleaningZones, submitRequest, timeAndEvaluator } = props;
  const { minsFromNow, evaluator, setTimeError, setMinsFromNow } = timeAndEvaluator;
  const [allZones, setZones] = React.useState(cleaningZones);
  const [targetZone, setTargetZone] = React.useState('');

  //errors
  const [zoneError, setZoneError] = React.useState("");
  
  const classes = useFormStyles();
  
  React.useEffect(() => {
    setZones(cleaningZones);
  }, [cleaningZones]);

  const cleanUpForm = () => {
    setMinsFromNow(0);
    setTargetZone('');
  }

  const isFormValid = () => {
    if(minsFromNow < 0) {
      setTimeError("Start time cannot be negative");
      return false;
    }
    if(targetZone.length === 0) {
      setZoneError("Cleaning zone cannot be an empty field");
      return false;
    }
    return true;
  }

  const createRequest = () => {
    let start_time = minsFromNow;
    let cleaning_zone = targetZone;
    let request = {};
    if (evaluator.length > 0 ){
      let evaluator_option = evaluator;
      request = { task_type: "Clean",
                  start_time: start_time,
                  evaluator: evaluator_option,
                  description: {'cleaning_zone': cleaning_zone} }
    } else {
      request = { task_type: "Clean",
                  start_time: start_time,
                  description: {'cleaning_zone': cleaning_zone} }
      }
    return request;
  }

  const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      let request = createRequest();
      submitRequest(request, "Cleaning");
      cleanUpForm();
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
                />
            </div>
            <div className={classes.buttonContainer}>
                <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Request</Button>
            </div>
        </Box>
    );
} 

export default CleaningForm;