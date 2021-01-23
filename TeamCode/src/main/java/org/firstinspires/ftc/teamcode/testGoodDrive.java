package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(group="chrisBot", name="testGoodDrive")
public class testGoodDrive extends LinearOpMode {
    chrisBot robot = new chrisBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        telemetry.setAutoClear(true);

        while(!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            telemetry.addData("Mode", "calibrating");
            telemetry.update();
        }
        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        robot.stemPIDdrive(5000, 0.5);

        robot.setAllPower(0);
    }
}
