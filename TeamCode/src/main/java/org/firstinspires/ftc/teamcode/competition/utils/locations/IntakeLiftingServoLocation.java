package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardServo;

public class IntakeLiftingServoLocation extends Location {

    private final StandardServo SERVO;

    public IntakeLiftingServoLocation(HardwareMap hardware) {
        SERVO = new StandardServo(hardware, hardware.appContext.getString(R.string.INTAKE_LIFTING_SERVO));
    }

    @Override
    public void stop() {
        SERVO.stop();
    }

    @Override
    public boolean isInputLocation() {
        return true;
    }

    @Override
    public boolean isOutputLocation() {
        return false;
    }

    @Override
    public InteractionSurface getInternalInteractionSurface() {
        return SERVO;
    }

}
