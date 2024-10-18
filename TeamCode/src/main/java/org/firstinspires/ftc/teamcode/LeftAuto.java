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

        //arm position in ticks
        //45 degrees=700
        //0=horizontal
        //extend pos is in inches(2 digits)

        //drives forward and puts the specimen on the hook
        robot.encoderDrive(1,15);
        robot.setArmPos(750);
        robot.setExtendPos(12.75);
        robot.setArmPos(450);
        robot.setExtendPos(0);
        robot.setArmPos(0);
        //drives to the samples
        robot.encoderStrafe(1, 32);
        //turns around
        robot.encoderTurn(1,180);
        //sample1

        robot.encoderDrive(1,-48);
        robot.encoderStrafe(1, -14);
        robot.setArmPos(-450);
        robot.setExtendPos(5.0);
        robot.runIntake();
        sleep(1000);
        robot.encoderDrive(1, 30);
        robot.setExtendPos(0);
        robot.setArmPos(700);
        robot.encoderTurn(1,45);
        robot.encoderDrive(1,12.8);
        robot.setArmPos(1000);
        robot.setExtendPos(-3550);
        robot.runOuttake();
        sleep(1000);
        robot.setArmPos(1024);
        robot.setExtendPos(0);

//        //sample2
//        robot.encoderDrive(1,-48);
//        robot.encoderTurn(1,-12);
//        robot.encoderStrafe(1,-12.2);
//        robot.encoderDrive(1,45);
//        //sample3
//        robot.encoderDrive(1,-45);
//        robot.encoderStrafe(1,-12);
//        robot.encoderDrive(1,43.5);
    }
}