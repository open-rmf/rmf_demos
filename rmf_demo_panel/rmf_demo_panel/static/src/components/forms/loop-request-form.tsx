import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Typography from '@material-ui/core/Typography';
import Autocomplete, { AutocompleteRenderInputParams } from '@material-ui/lab/Autocomplete';
import { useFormStyles } from '../styles';

interface LoopDescription {
  num_loops: number,
  start_name: string,
  finish_name: string
}

interface LoopFormProps {
  availablePlaces: string[]
  submitRequest: (request: {}, type: string) => void;
  timeAndEvaluator: { minsFromNow: number, evaluator: string, setTimeError: React.Dispatch<React.SetStateAction<string>>, setMinsFromNow: React.Dispatch<React.SetStateAction<number>>}
}

const LoopRequestForm = (props: LoopFormProps): React.ReactElement => {
  const { availablePlaces, submitRequest, timeAndEvaluator } = props;
   const { minsFromNow, evaluator, setTimeError, setMinsFromNow } = timeAndEvaluator;
  const classes = useFormStyles();
  const [startLocation, setStartLocation] = React.useState("");
  const [endLocation, setEndLocation] = React.useState("");
  const [places, setPlaces] = React.useState(availablePlaces);
  const [numLoops, setNumLoops] = React.useState(1);

  //errors
  const [numLoopsError, setNumLoopsError] = React.useState("");
  const [locationError, setLocationError] = React.useState("");

  React.useLayoutEffect(() => {
    setPlaces(availablePlaces);
  }, [availablePlaces]);

  const isFormValid = (): boolean => {
    let isValid = true;
    if(startLocation == endLocation) {
      setLocationError("Start and end locations cannot be the same");
      isValid = false;
    }
    if(startLocation == '' || endLocation == '') {
      setLocationError('Please select a location');
      isValid = false;
    }
    if(numLoops <= 0) {
      setNumLoopsError("Number of loops can only be > 0");
      isValid = false;
    }
    if(minsFromNow < 0) {
      setTimeError("Start time cannot be negative");
      isValid = false;
    }
    return isValid;
  }

  const cleanUpForm = () => {
    setStartLocation("");
    setEndLocation("");
    setNumLoops(1);
    setMinsFromNow(0);
    cleanUpError();
  }

  const cleanUpError = () => {
      setLocationError('');
      setNumLoopsError('');
      setTimeError('');
  };

  const createRequest = () => {
    let description: LoopDescription = {
      num_loops: numLoops,
      start_name: startLocation,
      finish_name: endLocation,
    }
    let start_time = minsFromNow;
    let request = {};
    if (evaluator.length > 0 ) {
        let evaluator_option = evaluator;
        request = { task_type: "Loop",
                    start_time: start_time,
                    evaluator: evaluator_option,
                    description: description }
    } else {
      request = { task_type: "Loop",
                  start_time: start_time,
                  description: description }
    }
    return request;
  }
    
  const handleSubmit = (ev: React.FormEvent): void => {
    ev.preventDefault();
    if(isFormValid()) {
      let request = createRequest();
      submitRequest(request, "Loop");
      cleanUpForm();
    }
  }

  return (
        <Box className={classes.form} role="loop-request-form">
            <div className={classes.divForm}>
            <Typography variant="h6">Schedule a Loop Request</Typography>
                <Autocomplete
                options={places}
                getOptionLabel={(place) => place}
                id="set-start-location"
                openOnFocus
                onChange={(_, value) => setStartLocation(value)}
                renderInput={(params: AutocompleteRenderInputParams) => 
                  <TextField {...params} 
                    label="Select start location" 
                    variant="outlined" 
                    margin="normal"
                    error={!!locationError}
                    helperText={locationError} 
                  />}
                value={startLocation ? startLocation : null}
                />
            </div>
            <div className={classes.divForm}>
                <Autocomplete id="set-end-location"
                openOnFocus
                options={places}
                getOptionLabel={(place) => place}
                onChange={(_, value) => setEndLocation(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select end location" variant="outlined" margin="normal" error={!!locationError} helperText={locationError}/>}
                value={endLocation ? endLocation : null}
                />
            </div>
            <div className={classes.divForm}>
                <TextField
                  className={classes.input}
                  onChange={(e) => {
                  setNumLoops(e.target.value ? parseInt(e.target.value) : 0);
                  }}
                  placeholder="Set number of loops"
                  type="number"
                  value={numLoops || ''}
                  label="Number of Loops"
                  variant="outlined"
                  id="set-num-loops"
                  error={!!numLoopsError}
                  helperText={numLoopsError}
                />
            </div>
            <div className={classes.buttonContainer}>
                <Button variant="contained" color="primary" onClick={handleSubmit} className={classes.button}>Submit Request</Button>
            </div>
        </Box>
    );
}

export default LoopRequestForm;
