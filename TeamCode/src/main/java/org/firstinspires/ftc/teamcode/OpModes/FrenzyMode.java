package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;

@TeleOp(name = "Frenzy", group = "Robot15173")
public class FrenzyMode extends LinearOpMode {

    // Declare OpMode Members
    FrenzyBot robot = new FrenzyBot();
    private ElapsedTime runtime = new ElapsedTime();

    // Locator Variables
    RobotCoordinatePosition locator = null;

    @Override
    public void runOpMode() {
        try {
            try{
                robot.init(this, this.hardwareMap, telemetry);
            } catch (Exception ex) {
                telemetry.addData("Init", ex.getMessage());
            }
            telemetry.update();

            locator = new RobotCoordinatePosition(robot, RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                // DRIVING
                double drive = gamepad1.left_stick_y;
                double turn = 0;
                double ltrigger = gamepad1.left_trigger;
                double rtrigger = gamepad1.right_trigger;
                if (ltrigger > 0) {
                    turn = -ltrigger;
                } else if (rtrigger > 0) {
                    turn = rtrigger;
                }

                double strafe = gamepad1.right_stick_x;

                if (Math.abs(strafe) > 0) {
                    telemetry.addData("Strafing", "Left: %2f", strafe);
                    telemetry.update();
                    if (strafe < 0) {
                        robot.strafeRight(Math.abs(strafe));
                    } else {
                        robot.strafeLeft(Math.abs(strafe));
                    }
                } else {
                    robot.move(drive, turn);
                }

                telemetry.addData("X ", locator.getXInches());
                telemetry.addData("Y ", locator.getYInches());
                telemetry.addData("Heading (Degrees)", locator.getOrientation());
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(1000);
        }
        finally {
            if (locator != null){
                locator.stop();
            }
        }
    }
}
