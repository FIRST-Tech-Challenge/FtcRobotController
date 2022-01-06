package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardTouchSensor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

public class ElevatorBottomLimitSwitchLocation extends Location {

    public enum Values {
        PRESSED,
        FORCE
    }

    private StandardTouchSensor SENSOR;

    public ElevatorBottomLimitSwitchLocation(HardwareMap hardware) {
        try {
            SENSOR = new StandardTouchSensor(hardware, Resources.Elevator.Sensors.BottomLimitSwitch);
        } catch(Exception ignored) {}
    }

    public double returnOutput(Values values) {
        if(SENSOR == null) {
            return 0;
        }
        switch(values) {
            case PRESSED:
                return SENSOR.isPressed() ? 1 : 0;
            case FORCE:
                return SENSOR.getForce();
            default:
                return 0;
        }
    }

    @Override
    public void stop() {
        if(SENSOR == null) {
            return;
        }
        SENSOR.stop();
    }

    @Override
    public boolean isInputLocation() {
        return false;
    }

    @Override
    public boolean isOutputLocation() {
        return true;
    }

    @Override
    public InteractionSurface getInternalInteractionSurface() {
        return SENSOR;
    }

}
