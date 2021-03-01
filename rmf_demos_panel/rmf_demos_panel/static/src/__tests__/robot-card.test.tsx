import React from 'react';
import { render, screen } from '@testing-library/react';
import { RobotCard } from '../components/robots/robot-card';

describe('Robot Card', () => {
     const exampleState  = {
            robot_name: "magnus",
            fleet_name: "fleet1",
            assignments: "1003",
            mode: "Idle-0",
            battery_percent: "99.99998474121094",
            level_name: "Level 2"
        };

    test("should render", () => {
        render(<RobotCard robotState={exampleState} />);
        expect(screen.getByRole('robot-details')).toBeInTheDocument();
    });
});
