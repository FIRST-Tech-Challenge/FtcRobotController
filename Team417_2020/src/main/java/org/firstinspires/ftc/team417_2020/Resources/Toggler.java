package org.firstinspires.ftc.team417_2020.Resources;

public class Toggler {
    public boolean toggleState = false;
    public boolean prevState = false;
    public boolean press = false;

    public Toggler() {

    }

    public boolean toggle(boolean button)
    {
        // if button is pressed then alternate the toggle state (this is the value returned to us)
        /*
        if(press)
        {
            toggleState = !toggleState;
        } */
        // if prevState was false and the button is pushed we now know the button is pressed

        // if the button is pressed (press is true) then toggle press to false because the button has been pushed
        /*
        else if(press)
        {
            press = false;
        } */
        // set previous state to the current push of the button

        if(!prevState && button)
        {
            //press = true;
            toggleState = !toggleState;
        }

        prevState = button;

        return toggleState;
    }

    public boolean getToggleState() {
        return toggleState;
    }
}
