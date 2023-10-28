package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Gamepad")
public class GamepadOpmode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        int currentTarget = 0;
        boolean holdPosition = false;
        int armLimit = 4500;
        double rollerVelocity = 0d;

        robot.init(hardwareMap);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("roller", () -> String.format("%.1f", robot.rollerMotor.getVelocity()));
        telemetry.addData("arm", () -> String.format("%d", robot.armMotor.getCurrentPosition()));
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region arm control
            int currentArmPosition = robot.armMotor.getCurrentPosition();

            if (gamepad1.left_trigger > 0d) { // raise lift
                robot.armMotor.setTargetPosition(armLimit);
                robot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0d) { // lower lift
                double powerScale = currentArmPosition > 0 ? 1d : 0.02d;
                robot.setArmMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(-gamepad1.right_trigger * powerScale);
            } else { // stop lift
                robot.armMotor.setPower(0d);
            }
            //endregion

            //region a button
            if (gamepad1.a && !a_pressed) {
            }
            //endregion

            //region roller
            if (gamepad1.left_bumper && !lb_pressed) {
                rollerVelocity = rollerVelocity == 0d ? 2700d : 0d;
            }

            if (gamepad1.right_bumper && !rb_pressed) {
                rollerVelocity = rollerVelocity == 0d ? -2000d : 0d;
            }
            robot.rollerMotor.setVelocity(rollerVelocity);
            //endregion

            //region drivetrain control
            double sensitivity = 2.0; // Adjust the sensitivity value as needed
            double deadzone = 0.1; // Adjust the deadzone value as needed

            double drive = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x;
            double side = gamepad1.right_stick_x;

            // Apply deadzone to the input values
            if (Math.abs(drive) < deadzone) {
                drive = 0.0;
            }

            if (Math.abs(turn) < deadzone) {
                turn = 0.0;
            }

            if (Math.abs(side) < deadzone) {
                side = 0.0;
            }

            // Apply the power function to the input values
            drive = Math.signum(drive) * Math.pow(Math.abs(drive), sensitivity);
            turn = Math.signum(turn) * Math.pow(Math.abs(turn), sensitivity);
            side = Math.signum(side) * Math.pow(Math.abs(side), sensitivity);

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
