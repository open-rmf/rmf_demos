import React from 'react';
import { render, screen } from '@testing-library/react';
import NavTabs from '../components/fixed-components/tabs';

describe('Nav Tabs', () => {
    test("should render", () => {
        let handleWorldChange = jest.fn();
        render(<NavTabs handleWorldChange={handleWorldChange}/>);
        expect(screen.getByRole('nav-tabs')).toBeVisible();
    });
})