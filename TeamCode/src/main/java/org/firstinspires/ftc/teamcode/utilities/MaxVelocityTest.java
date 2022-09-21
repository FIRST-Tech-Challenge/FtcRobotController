/*
Max Velocity Test for PIDF Tuning
https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit
Get maxVelocity by running this OpMode

F = 32767 / maxVelocity
P = 0.1 * F
I = 0.1 * P
D = 0
positionP = 5.0

 */
package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name="Max Velocity Test", group="Utilities")
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        waitForStart();

        motor.setPower(1.0);

        while (opModeIsActive()) {
            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            //intake in
            // motor.setPower(gamepad2.left_trigger * 0.75);

            //intake out
            // motor.setPower(-gamepad2.right_trigger * 0.3);

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
