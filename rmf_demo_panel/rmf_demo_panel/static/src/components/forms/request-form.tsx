import React from "react";
import Box from '@material-ui/core/Box';
import TextField from '@material-ui/core/TextField';
import Typography from '@material-ui/core/Typography';
import Autocomplete, { AutocompleteRenderInputParams } from '@material-ui/lab/Autocomplete';
import { useFormStyles } from "../styles";
import CleaningForm from './cleaning-form';
import LoopRequestForm from './loop-request-form';
import DeliveryForm from './delivery-form';
import { WorldContext } from '../fixed-components/app-context';
import { submitRequest } from '../services';

const RequestForm = (): React.ReactElement => {
    const { config } = React.useContext(WorldContext);
    const [requestTypes, setRequestTypes] = React.useState(config.valid_task);
    const [formType, setFormType] = React.useState('');
    const [loopPlaces, setLoopPlaces] = React.useState([]);
    const [deliveryOptions, setDeliveryOptions] = React.useState({});
    const [cleaningZones, setCleaningZones] = React.useState([]);
    const [minsFromNow, setMinsFromNow] = React.useState(0);
    const [evaluator, setEvaluator] = React.useState('');
    
    const [timeError, setTimeError] = React.useState("");

    React.useEffect(() => {
        if(Object.keys(config).length > 0) {
            setRequestTypes(config.valid_task);
            setLoopPlaces(config.task.Loop.places);
            setDeliveryOptions(config.task.Delivery.option);
            setCleaningZones(config.task.Clean.option);
        } else {
            setRequestTypes([]);
            setFormType('');
            setLoopPlaces([]);
            setDeliveryOptions({});
            setCleaningZones([]);
        }
    }, [config]);

    const returnFormType = (formType: string) => {
        const timeAndEvaluator = { minsFromNow, evaluator, setTimeError, setMinsFromNow };
        switch (formType) {
            case "Loop":
                return <LoopRequestForm availablePlaces={loopPlaces} submitRequest={submitRequest} timeAndEvaluator={timeAndEvaluator}/>
            case "Delivery": 
                return <DeliveryForm deliveryOptions={deliveryOptions} submitRequest={submitRequest} timeAndEvaluator={timeAndEvaluator}/>
            case "Clean":
                return <CleaningForm cleaningZones={cleaningZones} submitRequest={submitRequest} timeAndEvaluator={timeAndEvaluator}/>
        }
    }

    const evaluators:string[] = ["lowest_delta_cost", "lowest_cost", "quickest_time"];
  
    const classes = useFormStyles();
    
    return (
        <Box className={classes.form}>
            <div className={classes.divForm}>
            <Typography variant="h6">Submit a Task</Typography>
                <Autocomplete
                options={requestTypes}
                getOptionLabel={(requestType) => requestType}
                id="set-form-type"
                openOnFocus
                defaultValue={formType}
                onChange={(_, value) => setFormType(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Select a request type" variant="outlined" margin="normal" />}
                />
            </div>
            <div className={classes.divForm}>
                <TextField
                className={classes.input}
                onChange={(e) => {
                setMinsFromNow(e.target.value ? parseInt(e.target.value) : 0);
                }}
                placeholder="Set start time (mins from now)"
                type="number"
                value={minsFromNow || 0}
                label="Set start time (mins from now)"
                variant="outlined"
                id="set-start-time"
                error={!!timeError}
                helperText={timeError}
                />
            </div>
            <div className={classes.divForm}>
                <Autocomplete id="set-evaluator"
                openOnFocus
                options={evaluators}
                getOptionLabel={(evaluator) => evaluator}
                onChange={(_, value) => setEvaluator(value)}
                renderInput={(params: AutocompleteRenderInputParams) => <TextField {...params} label="Choose an evaluator (optional)" variant="outlined" margin="normal" />}
                />
            </div>
            <div className={classes.divForm}>
                {formType && returnFormType(formType)}
            </div>
        </Box>
    )
}

export default RequestForm;
