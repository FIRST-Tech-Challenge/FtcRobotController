package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Left-auto", group="auto")
public class LeftAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Bot robot = new Bot(this);

        robot.init(hardwareMap);

        waitForStart();

        robot.encoderDrive(1,15);
//        robot.setArmPos(45);
//        robot.setExtendPos(5);
//        robot.runOuttake();
//        sleep(5000);
        robot.encoderStrafe(1, 32);
        robot.encoderTurn(1,180);
        robot.encoderDrive(1,-35.8);
        robot.encoderStrafe(1, -12);
        robot.encoderTurn(1,12);
        robot.encoderDrive(1,48);
        robot.encoderDrive(1,-48);
        robot.encoderTurn(1,-12);
        robot.encoderStrafe(1,-12.2);
        robot.encoderDrive(1,45);
        robot.encoderDrive(1,-45);
        robot.encoderStrafe(1,-12);
        robot.encoderDrive(1,43.5);
    }
}