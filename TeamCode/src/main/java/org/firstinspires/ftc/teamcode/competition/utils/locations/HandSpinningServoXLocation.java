package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardServo;

public class HandSpinningServoXLocation extends Location {

    private final StandardServo SERVO;

    public HandSpinningServoXLocation(HardwareMap hardware) {
        SERVO = new StandardServo(hardware, hardware.appContext.getString(R.string.HAND_SPINNING_SERVO_X));
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
