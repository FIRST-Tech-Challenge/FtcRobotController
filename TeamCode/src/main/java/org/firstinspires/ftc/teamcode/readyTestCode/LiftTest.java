package org.firstinspires.ftc.teamcode.readyTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Lift Test Code")
public class LiftTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            // creates variables for right and left trigger values
            double rTrigger = gamepad1.right_trigger/5;
            double lTrigger = gamepad1.left_trigger/5;

            // holding down right trigger makes lift move up
            // and left trigger makes lift move down
            // depending on how much you press
            // while you aren't pressing anything, sets the motor power to zero
            if (rTrigger > 0) {
                lift.setPower(rTrigger);
            }else if (lTrigger > 0) {
                lift.setPower(-lTrigger);
            }else {
                lift.setPower(0);
            }

        }
    }
}