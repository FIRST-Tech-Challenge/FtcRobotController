package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tools.DeadzoneSquare;

public class SlideLift {
    private final DcMotor slideMotor;
    private final double MAX_ENCODER_VAL = 50.0;
    private final double MAX_LIFT_POWER = 0.5;
    private final DeadzoneSquare deadzoneObj = new DeadzoneSquare(0.1, 0.25, MAX_LIFT_POWER);

    public SlideLift(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
    }

    public void resetEncoders()
    {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setLiftPower (double joystick, TouchSensor touchDown) {
        double powerVal = deadzoneObj.computePower(joystick);
        if (touchDown.isPressed() && powerVal<0) {
            slideMotor.setPower(0);
        }
        else if (slideMotor.getCurrentPosition() >= MAX_ENCODER_VAL && powerVal>0) {
            slideMotor.setPower(0);
        }
        else {
            slideMotor.setPower(powerVal);
        }
    }
}
