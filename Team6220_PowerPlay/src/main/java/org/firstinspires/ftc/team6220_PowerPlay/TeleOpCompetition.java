package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "TeleOpTest", group = "Test")

public class TeleOpCompetition extends BaseTeleOp{

    @Override public void runOpMode() {
        initHardware();
        waitForStart();
        //get angle after startup to prevent jitter on startup
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        openGrabber(true);
        while (opModeIsActive()) {
            //define what controls to use for their respective variables
            //(x movement, y movement, and turning)
            driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            idle();

        }
    }
}
