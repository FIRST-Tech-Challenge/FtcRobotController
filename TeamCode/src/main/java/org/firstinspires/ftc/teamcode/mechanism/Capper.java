package org.firstinspires.ftc.teamcode.mechanism;

import static org.firstinspires.ftc.teamcode.Constants.CAPPER_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Capper implements Mechanism {
    public DcMotorEx arm;
    float targetPosition = 0;
    boolean onEncoders = true;
    @Override
    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "capper");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void run(Gamepad gamepad) {
        if(gamepad.y) {
            // Ability for manual control, which resets the motor's encoder value when done
            if(onEncoders) {
                onEncoders = false;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            arm.setPower(-gamepad.right_stick_y * 0.6);
        } else {
            if(!onEncoders) {
                // Resetting the encoder value
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                targetPosition = 0;
                onEncoders = true;
            }
            targetPosition -= gamepad.right_stick_y * 10;
            targetPosition = Range.clip(targetPosition, 0, 1475);
            goTo((int) targetPosition, CAPPER_SPEED);
        }
    }

    public void goTo(int position, double power){
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
    }
}
