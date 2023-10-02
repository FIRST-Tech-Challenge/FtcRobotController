package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Gamepad")
public class GamepadOpmode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region drivetrain control
            double drive = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x;
            double side = gamepad1.right_stick_x;

            double pLeftFront = Range.clip(drive + turn + side, -1.0d, 1.0d);
            double pLeftRear = Range.clip(drive + turn - side, -1.0d, 1.0d);
            double pRightFront = Range.clip(drive - turn - side, -1.0d, 1.0d);
            double pRightRear = Range.clip(drive - turn + side, -1.0d, 1.0d);

            // Send calculated power to wheels
            robot.setDrivePower(pLeftFront, pLeftRear, pRightFront, pRightRear);
            //endregion

            gamepadUpdate();
            telemetry.update();
        }
    }
}
