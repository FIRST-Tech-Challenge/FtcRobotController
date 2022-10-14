package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

@TeleOp(name = "VSLAM Odometry Test", group = "Robot15173")
//@Disabled
public class VSlamOdometryTest extends LinearOpMode {

    IBaseOdometry odometry = null;

    @Override
    public void runOpMode() {
        try {
            int startXInches = 50;
            int startYInches = 15;
            int startHeading = 180;

            odometry =  VSlamOdometry.getInstance(this.hardwareMap, 20, startXInches, startYInches, startHeading);

            Thread odometryThread = new Thread(odometry);
            odometryThread.start();

            telemetry.addData("VSLAM", "VSlam Ready");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("X", "%.3f", odometry.getCurrentX());
                telemetry.addData("Y", "%.3f", odometry.getCurrentY());
                telemetry.addData("Heading", "%.3f", odometry.getOrientation());
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
            sleep(10000);
        } finally {
            if (odometry != null) {
                odometry.stop();
            }
        }

    }
}
