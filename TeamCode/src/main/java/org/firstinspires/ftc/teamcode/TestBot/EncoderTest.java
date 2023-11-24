package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Encoder")

public class EncoderTest extends LinearOpMode {
    TestHardware robot = new TestHardware();
    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();

            robot.turnOnEncoders();
            robot.encoderDrive(30);
            sleep(250);
            robot.encoderStrafe(20);
            sleep(250);
            robot.encoderStrafe(-20);
            sleep(250);
            robot.encoderDrive(-20);
            sleep(250);



    }
}
