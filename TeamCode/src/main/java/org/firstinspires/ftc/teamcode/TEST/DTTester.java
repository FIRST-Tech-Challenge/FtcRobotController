package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.middleend.HardwareMappings.HMap2;

@Autonomous(name = "DTTester")
public class DTTester extends LinearOpMode {
    HMap2 robo = new HMap2();

    @Override
    public void runOpMode() throws InterruptedException {
        robo.init(hardwareMap);
        // Code for calibrating the imu
        while (!isStopRequested() && !robo.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Calibration: ", "Done");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robo.drive(0.6, 0.40);
        }
    }
}
