package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.FrenzyBotI;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

@Disabled
@TeleOp(name = "OdometryFeedback", group = "Robot15173")
public class OdometryFeedbackMode extends LinearOpMode {
    FrenzyBotI robot = new FrenzyBotI();
    IBaseOdometry odometry = null;
    private ElapsedTime runtime = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        try {
            robot.init(this, this.hardwareMap, telemetry);
        } catch (Exception ex) {
            telemetry.addData("Robot Init", ex.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }

        try {
            // VSLAM odometry
            odometry = VSlamOdometry.getInstance(this.hardwareMap);
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

            telemetry.addData("Odometry", "Started");
            telemetry.update();

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
                        if (strafe < 0) {
                            robot.strafeRight(Math.abs(strafe));
                        } else {
                            robot.strafeLeft(Math.abs(strafe));
                        }
                    } else {
                        telemetry.addData("Moving", "Drive: %2f, Turn: %.2f", drive, turn);
                        robot.move(drive, turn);
                    }

                    double xPos = odometry.getCurrentX();
                    double yPos = odometry.getCurrentY();
                    double aDegrees = odometry.getAdjustedCurrentHeading();

                    telemetry.addData("X ", String.format("%.2f", xPos));
                    telemetry.addData("Y ", String.format("%.2f", yPos));
                    telemetry.addData("H", aDegrees);
                    telemetry.update();


                } catch (Exception ex) {
                    telemetry.addData("Issues with the OpMode", ex.getMessage());
                    telemetry.update();
                    sleep(10000);
                }
            }
        } finally {
            odometry.stop();
        }
    }
}
