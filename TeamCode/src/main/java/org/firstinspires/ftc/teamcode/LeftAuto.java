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

        //drive less for sample
        //turn more drive less

        //drives forward and gets a 5.0 gpa
        robot.encoderDrive(1,19.43);
        robot.setArmPos(1045);
        robot.setExtendPos(13.948);
        robot.setArmPos(450);
        robot.setExtendPos(0.25);
        robot.setArmPos(25);
        //drives to the samples
        robot.encoderStrafe(1, 33);
        robot.encoderTurn(1,180);
        robot.encoderDrive(1,-32.5);
        sleep(250);
        robot.encoderStrafe(1, -15.5314);
        sleep(500);
        //sample 1
        robot.encoderDrive(1, 4.37);
        robot.autoIntake(2.0);
        robot.encoderStrafe(1,5.5);
        robot.encoderDrive(1, 29.5);
        robot.setExtendPos(0.25);
        robot.setArmPos(1150);
        robot.encoderTurn(1,50);
        robot.setArmPos(1175);
        robot.setExtendPos(10.0);
        robot.encoderDrive(1,15);
        robot.runIntakeForTime(1.25, -1);
        robot.setArmPos(1024);
        robot.setExtendPos(0.25);
        robot.setArmPos(25);

        robot.encoderDrive(1, 6.8);
        robot.encoderDrive(1, -1);

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