import { makeStyles } from '@material-ui/core';

export const useHeaderStyles = makeStyles({
    centralise: {
        paddingTop: '1rem',
        paddingBottom: '1rem',
        margin: "auto"
    }
});

export const useFooterStyles = makeStyles({
   footer: {
       backgroundColor: "#FAFAFA",
       paddingTop: "1em",
       paddingBottom: "1em",
       textAlign: "center"
   }
});

export const usePanelContainerStyles = makeStyles(theme => ({
   panels: {
       paddingTop: "1em",
       paddingBottom: "2em"
   },
    centered: {
      padding: theme.spacing(2),
      textAlign: 'center',
      color: theme.palette.text.secondary,
    }
}));

export const useContainerStyles = makeStyles({
    container: {
        paddingTop: "1em",
        paddingBottom: "1em",
        maxHeight: "75vh",
        verticalAlign: "top",
        overflow: "auto",
        minHeight: 0
    },
    grid: {
        overflow: "hidden",
        flexDirection: "row",
        verticalAlign: "top",
    }
});

export const useFormStyles = makeStyles({
  form: {
    display: 'flex',
    flexDirection: 'column',
    width: '100%',
    verticalAlign: "top",
    paddingTop: "1em",
    paddingBottom: "1em"
  },
  divForm: {
    width: '100%',
    marginTop: 0,
    marginBottom: 0,
    alignItems: 'center'
  },
  input: {
    width: '100%',
    marginTop: '1em',
  },
  label: {
    width: '100%',
  },
  fileInput: {
    width: '100%',
    display: 'none'
  },
  selectButton: {
    width: '50%'
  },
  button: {
    width: '70%',
  },
  buttonContainer: {
    marginTop: '0.7rem',
    marginBottom: '0.7rem',
    width: '100%',
    alignItems: 'center'
  },
});

export const useTaskCardStyles = makeStyles({
  root: {
    maxWidth: 240,
    margin: "0.25em",
    textAlign: 'center',
    justifyContent: 'center'
  },
});

export const useRobotCardStyles = makeStyles({
  root: {
    maxWidth: 240,
    maxHeight: 280,
    overflow: "auto",
    margin: "0.5em"
  },
  text: {
    textAlign: "center"
  }
});
