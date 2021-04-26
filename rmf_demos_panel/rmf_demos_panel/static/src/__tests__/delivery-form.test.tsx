import React from 'react';
import { cleanup, render, screen, fireEvent, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import DeliveryForm from '../components/forms/delivery-form';

describe('Delivery Form', () => {
    let root: ReturnType<typeof renderForm>;
    let submitRequest = jest.fn();
    let setTimeError = jest.fn();
    let setMinsFromNow = jest.fn();
    let setPriority = jest.fn();
    let setPriorityError = jest.fn();
    let minsFromNow = 0;
    let priority = 0;
    let timeAndPriority = { minsFromNow, priority, setTimeError, setMinsFromNow, setPriority, setPriorityError }
    
    function renderForm() {
        const deliveryOptions = {
            "mop": {
              "pickup_place_name": "mopcart_pickup",
              "pickup_dispenser": "mopcart_dispenser",
              "dropoff_place_name": "spill",
              "dropoff_ingestor": "mopcart_collector"
            }
        }
        return render(<DeliveryForm deliveryOptions={deliveryOptions} submitRequest={submitRequest} timeAndPriority={timeAndPriority}/>);
    }

    beforeEach(() => {
        root = renderForm();
    });

    afterEach(() => {
        root.unmount();
        cleanup();
    });

    it("should render", () => {
        expect(screen.getByRole('delivery-form')).toBeInTheDocument();
    });

    it("should render error messages when invalid form is submitted", () => {
        const submitButton = screen.getByText('Submit Request');
        userEvent.click(submitButton);
        expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    });

    it("should submit a valid form", () => {
        const submitButton = screen.getByText('Submit Request');
        userEvent.click(root.getByLabelText('Select delivery task'));
        userEvent.click(within(screen.getAllByRole('listbox')[0]).getByText('mop'));
        userEvent.click(submitButton);
        expect(submitRequest).toHaveBeenCalled();
    });
});
