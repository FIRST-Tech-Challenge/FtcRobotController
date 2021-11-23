package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Lift implements Mechanism {
    public DcMotorEx liftMotor;
    float targetPosition = 0;
    boolean onEncoders = true;
    @Override
    public void init(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void run(Gamepad gamepad) {
        if(gamepad.y) {
            // Ability for manual control, which resets the motor's encoder value when done
            if(onEncoders) {
                onEncoders = false;
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            liftMotor.setPower(-gamepad.left_stick_y * 0.7);
        } else {
            if(!onEncoders) {
                // Resetting the encoder value
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                targetPosition = 0;
            }
            targetPosition -= gamepad.left_stick_y * 10;
            targetPosition = Range.clip(targetPosition, 0, 1500);
            goTo((int) targetPosition, 0.8);
        }
    }

    public void goTo(int position, double power){
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
    }
}
