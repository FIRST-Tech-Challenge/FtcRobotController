package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "OdometryFeedback", group = "Robot15173")
public class OdometryFeedbackMode extends LinearOpMode {
    FrenzyBot robot = new FrenzyBot();
    private ElapsedTime runtime = new ElapsedTime();

    IBaseOdometry odometry = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        try{
            robot.init(this, this.hardwareMap, telemetry);
        } catch (Exception ex) {
            telemetry.addData("Robot Init", ex.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }


        try {
            // VSLAM odometry
            odometry =  VSlamOdometry.getInstance(this.hardwareMap);
            odometry.setInitPosition(0, 0, 0);
        } catch (Exception ex) {
            telemetry.addData("Odometry Init", ex.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }

        try {
            Thread odometryThread = new Thread(odometry);
            odometryThread.start();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                try {
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

                    int robotRadius = 9; // inches
                    double xPos = odometry.getXInches();
                    double yPos = odometry.getYInches();
                    double aDegrees = odometry.getAdjustedCurrentHeading();
                    double aRadians = aDegrees * Math.PI / 180.0;

                    telemetry.addData("X ", xPos);
                    telemetry.addData("Y ", yPos);
                    telemetry.addData("H", aDegrees);
                    telemetry.update();

                    // Draw the robot on dashboard canvas
                    field.strokeCircle(xPos, yPos, robotRadius);
                    double arrowX = Math.cos(aRadians) * robotRadius, arrowY = Math.sin(aRadians) * robotRadius;
                    double x1 = xPos+ arrowX  / 2, y1 = yPos + arrowY / 2;
                    double x2 = xPos + arrowX, y2 = yPos + arrowY;
                    field.strokeLine(x1, y1, x2, y2);

                    dashboard.sendTelemetryPacket(packet);

                } catch (Exception ex) {
                    telemetry.addData("Issues with the OpMode", ex.getMessage());
                    telemetry.update();
                    sleep(10000);
                }
            }
        }
        finally {
            odometry.stop();
        }
    }
}
