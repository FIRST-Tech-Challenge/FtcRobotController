package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.team15091.Robot.grabberPosition;

@TeleOp(name = "Gamepad")
public class GamepadOpmode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        int currentTarget = 0;
        boolean holdPosition = false;
        int highPolePos = 1950;

        robot.init(hardwareMap);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region arm control
            int currentArmPosition = robot.armMotor.getCurrentPosition();

            if (gamepad1.left_trigger > 0d) { // raise lift
                robot.armMotor.setTargetPosition(highPolePos);
                robot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(gamepad1.left_trigger);
                currentTarget = currentArmPosition;
            } else if (gamepad1.right_trigger > 0d && // lower lift
                    robot.limitSwitch.getState() == true) { // when limit sensor not pressed
                double powerScale = currentArmPosition > 800 ? 1d : 0.4d;
                robot.setArmMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(-gamepad1.right_trigger * powerScale);
                currentTarget = currentArmPosition;
                holdPosition = false;
            } else { // stop lift
                if (holdPosition == false && robot.limitSwitch.getState() == false) { // limit sensor pressed
                    robot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    currentTarget = 0;
                } else {
                    robot.armMotor.setTargetPosition(currentTarget);
                    robot.setArmMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(currentArmPosition > 100 ? 0.8d : 0.3d);
                }
            }
            //endregion

            //region grabber servo control
            if (gamepad1.a && !a_pressed) {
                // close grabber
                robot.setGrabber(grabberPosition == 1 ? 0 : 1);
            }
            //endregion

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
