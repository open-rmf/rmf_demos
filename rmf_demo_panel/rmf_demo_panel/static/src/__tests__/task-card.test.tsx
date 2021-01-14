import React from 'react';
import { render, screen } from '@testing-library/react';
import { TaskCard } from '../components/tasks/task-card';

describe('Task Card', () => {
    const exampleState  = {
        task_id: "1003",
        description: "zone_x",
        robot_name: "magnus",
        state: "Active/Executing",
        task_type: "Clean",
        start_time: 1500,
        end_time: 1540,
        progress: "42"
    };

    test("should render", () => {
        render(<TaskCard taskState={exampleState} />);
        expect(screen.getByRole('task-details')).toBeInTheDocument();
    });
});
