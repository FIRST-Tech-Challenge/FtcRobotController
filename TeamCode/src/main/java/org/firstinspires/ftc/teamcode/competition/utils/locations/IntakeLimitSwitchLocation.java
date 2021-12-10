package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardTouchSensor;

public class IntakeLimitSwitchLocation extends Location {

    public enum Values {
        PRESSED,
        FORCE
    }

    private final StandardTouchSensor SENSOR;

    public IntakeLimitSwitchLocation(HardwareMap hardware) {
        SENSOR = new StandardTouchSensor(hardware, hardware.appContext.getString(R.string.INTAKE_LIFTING_LIMIT_SWITCH));
    }

    public double returnOutput(Values values) {
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
