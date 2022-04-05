package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

@TeleOp(name = "ThieveryTest")
public class ThieveryTest extends GenericOpModeTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        this.initOdometryServos();
        podServos.lower();
        StandardTrackingWheelLocalizer odometry = new StandardTrackingWheelLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Pos", odometry.getPoseEstimate());
            odometry.update();
            telemetry.update();
        }
    }
}
