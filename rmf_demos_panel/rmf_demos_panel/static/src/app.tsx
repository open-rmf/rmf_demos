import React from 'react';
import Divider from '@material-ui/core/Divider';
import Header from "./components/fixed-components/header";
import PanelsContainer from './components/panels-container';
import Footer from './components/fixed-components/footer';
import NavTabs from './components/fixed-components/tabs';
import { WorldContext, World } from './components/fixed-components/app-context';
import { getDashboardConfig } from './components/services';

export default function App(): React.ReactElement {
    const currWorld = React.useContext(WorldContext);
    const [currentWorld, setCurrentWorld] = React.useState(currWorld);
    const [worldName, setWorldName] = React.useState('');
    
    const setDefaultConfig = async () => {
        const defaultConfig = await getDashboardConfig();
        setCurrentWorld({map: World.Office, config: defaultConfig});
        setWorldName(defaultConfig.world_name);
    };

    React.useEffect(() => {
       setDefaultConfig();
    }, []);

    return (
        <div>
            <WorldContext.Provider value={currentWorld}>
                <Header />
                <NavTabs worldName={worldName} />
                <Divider variant="middle" />
                <PanelsContainer />
                <Divider variant="middle"/>
                <Footer />
            </WorldContext.Provider>
        </div>
    )
}
