package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ChrisBot Autonomous Left SIMPLE",group="chrisBot")
public class actualAutonSimplifiedv3 extends LinearOpMode {
    chrisBot robot = new chrisBot();
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        robot.shootOn(0.235);

        if(isStopRequested()) {
            robot.shootOff();
        }

        waitForStart();

        telemetry.clear();
        robot.breakTelemetry();
        telemetry.update();
        runtime.reset();


        robot.lineUp();

        sleep(500);

        robot.byWheelEncoderDrive(0,-3,0,-3,0.25);
        sleep(2000);

        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < 200) {
            robot.intakeOn();
        }
        robot.intakeOff();
        robot.shootOn(0.237);
        robot.byWheelEncoderDrive(0,-2,0,-2,0.1);
        sleep(6000);
        e.reset();
        while(e.milliseconds() < 230) {
            robot.intakeOn();
        }
        robot.intakeOff();
        robot.shootOn(0.24);
        robot.byWheelEncoderDrive(0,-1.5,0,-1.5,0.1);
        sleep(3000);
        e.reset();
        while(e.milliseconds() < 700) {
            robot.intakeOn();
        }
        robot.intakeOff();


        telemetry.addData("Auton done, time elapsed (ms)",runtime.milliseconds());
        telemetry.update();

        robot.shootOff();

        // End auton

        sleep(60000);
    }

}
