package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();

        waitForStart();

        while (opModeIsActive()) {
            //robot.straightBlockingFixHeading(48, false, 0.8);
            robot.setHeading(-90, 0.7);
            this.sleep(3000);
            robot.setHeading(90, 0.7);
            this.sleep(3000);
            robot.setHeading(0, 0.7);
            this.sleep(3000);
            //robot.straightBlockingFixHeading(84,false, 0.8);
            robot.setHeading(0, 0.7);
            break;
        }
    }
}
