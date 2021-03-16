package org.wheelerschool.robotics.lib;

public class StatefulButton {
    public boolean state;
    public boolean lastValue = false;

    public StatefulButton(boolean defaultState) {
        this.state = defaultState;
    }

    public boolean update(boolean currentValue) {
        boolean freshClick = false;
        if ((currentValue != lastValue) && currentValue) {
            this.state = !this.state;
            freshClick = true;
        }

        lastValue = currentValue;
        return freshClick;
    }
}
