import React from 'react';
import Divider from '@material-ui/core/Divider';
import Header from "./components/fixed-components/header";
import PanelsContainer from './components/panels-container';
import Footer from './components/fixed-components/footer';
import NavTabs from './components/fixed-components/tabs';
import { WorldContext, World } from './components/fixed-components/app-context';
import { getDefaultConfig } from './components/services';

export default function App(): React.ReactElement {
    const currWorld = React.useContext(WorldContext);
    const [currentWorld, setCurrentWorld] = React.useState(currWorld);
    
    const setDefaultConfig = async () => {
        const defaultConfig = await getDefaultConfig();
        setCurrentWorld({map: World.Office, config: defaultConfig});
    };

    React.useEffect(() => {
       setDefaultConfig();
    }, []);

    return (
        <div>
            <WorldContext.Provider value={currentWorld}>
                <Header />
                <NavTabs handleWorldChange={(world) => setCurrentWorld(world)} />
                <Divider variant="middle" />
                <PanelsContainer />
                <Divider variant="middle"/>
                <Footer />
            </WorldContext.Provider>
        </div>
    )
}
