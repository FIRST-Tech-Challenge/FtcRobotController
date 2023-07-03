// note: to change the tests of the code, go to line 39 and change the values


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Novak Djokovic is the GOAT of tennis.")
public class Opsareautonomousnow extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;
    private int leftPos;
    private int rightPos;


    @Override
    public void runOpMode() {
        // !!!! note: if we switch to opmode (not linearopmode), then the initialized code must be in a different method than the running code


        // setting motors
        left = hardwareMap.get(DcMotor.class, "leftMotor");
        right = hardwareMap.get(DcMotor.class, "rightMotor");

        // reset encoders
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPos = 0;
        rightPos = 0;

        // reverse one of the motors so they don't spin in opposite directions
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // both positive means that it moves straight by 2000 ticks (around 1.5 feet?)
        // one positive one negative means it turns. ex. -2000, 2000 means it turns left, while 2000, -2000 means it turns right
        drive(2000, 2000, 0.8);
        drive(-2000, 2000, 0.5);
    }

    private void drive(int leftTarget, int rightTarget, double speed) {

        // set desired target
        leftPos += leftTarget;
        rightPos += rightTarget;
        left.setTargetPosition(leftPos);
        right.setTargetPosition(rightPos);

        // move to target until target is reached
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // sets velocity for robot based on desired target
        left.setPower(speed);
        right.setPower(speed);

        while(opModeIsActive() && left.isBusy() && right.isBusy()) {
            // ensure that nothing happens while the robot is attempting to reach the target
            idle();

        }
    }

}
