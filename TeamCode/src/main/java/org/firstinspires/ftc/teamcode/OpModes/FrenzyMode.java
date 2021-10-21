package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

@TeleOp(name = "Frenzy", group = "Robot15173")
public class FrenzyMode extends LinearOpMode {

    // Declare OpMode Members
    FrenzyBot robot = new FrenzyBot();
    private ElapsedTime runtime = new ElapsedTime();
    IBaseOdometry odometry = null;


    @Override
    public void runOpMode() {
        try {
            try{
                robot.init(this, this.hardwareMap, telemetry);
                odometry =  VSlamOdometry.getInstance(this.hardwareMap, 20);
                odometry.setInitPosition(50, 15, 0);
                Thread odometryThread = new Thread(odometry);
                odometryThread.start();
            } catch (Exception ex) {
                telemetry.addData("Init", ex.getMessage());
            }
            telemetry.update();


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
                    if (strafe < 0) {
                        robot.strafeRight(Math.abs(strafe));
                    } else {
                        robot.strafeLeft(Math.abs(strafe));
                    }
                } else {
                    robot.move(drive, turn);
                }
                telemetry.addData("Left front", robot.getLeftOdometer());
                telemetry.addData("Right front", robot.getRightOdometer());
                telemetry.addData("X", odometry.getXInches());
                telemetry.addData("Y", odometry.getYInches());
                telemetry.addData("Heading", odometry.getAdjustedCurrentHeading());
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(1000);
        }
        finally {
            if (odometry != null) {
                odometry.stop();
            }
        }
    }
}
