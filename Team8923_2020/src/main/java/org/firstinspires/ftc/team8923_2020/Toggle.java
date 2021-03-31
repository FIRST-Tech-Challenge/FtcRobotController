package org.firstinspires.ftc.team8923_2020;

public class Toggle {
    public boolean toggleState = false;
    public boolean prevState = false;
    public boolean press = false;

    public Toggle() {

    }

    public boolean toggle(boolean button)
    {
        if(press)
        {
            toggleState = !toggleState;
        }
        if(! prevState && button)
        {
            press = true;
        }
        else if(press)
        {
            press = false;
        }
        prevState = button;

        return toggleState;
    }

    public boolean getToggleState() {
        return toggleState;
    }
}


