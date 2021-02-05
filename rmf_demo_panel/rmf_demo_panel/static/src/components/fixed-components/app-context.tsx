import React from 'react';

// TODO: Rethink if different world tabs are necessary (Office)
export const Worlds = {
    0: "Office",
    1: "Airport",
    2: "Clinic",
    3: "Hotel"
}

export enum World {
    Office = 0,
    Airport,
    Clinic,
    Hotel,
}

export type WorldContextType = {
    map: World,
    config: any
}

export const WorldContext = React.createContext<WorldContextType>({ map: World.Office, config: {} });
