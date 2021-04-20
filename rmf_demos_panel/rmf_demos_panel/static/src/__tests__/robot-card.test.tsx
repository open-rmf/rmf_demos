import React from 'react';
import { render, screen } from '@testing-library/react';
import { RobotCard } from '../components/robots/robot-card';

describe('Robot Card', () => {
    const exampleState  = {
        robot_name: "magnus",
        fleet_name: "fleet1",
        assignments: ["1003", "task2", "Loop3"],
        mode: "Idle-0",
        battery_percent: "99.99998474121094",
        level_name: "Level 2"
    };

    it("should render", () => {
        const root = render(<RobotCard robotState={exampleState} />);
        expect(screen.getByRole('robot-details')).toBeInTheDocument();
        root.unmount();
    });

    it("should render arrows in assigned tasks", () => {
        render(<RobotCard robotState={exampleState} />);
        expect(screen.getByText('1003 ➜task2 ➜Loop3')).toBeInTheDocument();
    });
});
