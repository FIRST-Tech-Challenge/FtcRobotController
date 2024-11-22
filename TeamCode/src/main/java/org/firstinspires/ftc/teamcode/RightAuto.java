package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Right-auto", group="auto")
public class RightAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Bot robot = new Bot(this);

        robot.init(hardwareMap);

        waitForStart();

        //drives forward and gets a compliment from Mrs. Nagao
        robot.encoderDrive(1,19.43);
        robot.setArmPos(1045);
        robot.setExtendPos(13.948);
        robot.setArmPos(450);
        robot.setExtendPos(0.25);
        robot.setArmPos(25);

        //pushes samples in
        robot.encoderStrafe(1, -29.8);
        robot.encoderTurn(1,-180);
        robot.encoderDrive(1,-36);
        robot.encoderStrafe(1, 9.8);
        robot.encoderDrive(1,47.5);
        robot.encoderDrive(1,-47.5);
        robot.encoderStrafe(1, 15);
        robot.encoderDrive(1,46);
        robot.encoderDrive(1,-46);
        robot.encoderStrafe(1, 13.25);
        robot.encoderDrive(1,46.1);
        robot.encoderDrive(1,-15);
        sleep(800);
        robot.runIntakeForTime(1.0, -1);

        //extra specimen
        robot.setArmPos(230);
        robot.encoderDrive(1, 20.3);
        robot.setExtendPos(0.2);
        robot.setArmPos(315);
        robot.runIntakeForTime(0.75, 1);
        robot.setArmPos(580);
        robot.encoderDrive(1, -6.9);
        robot.encoderStrafe(1, -70);
        robot.encoderTurn(1, 180);
        robot.setArmPos(1070);
        robot.setExtendPos(12.501);
        robot.encoderDrive(1,9.5);
        robot.setArmPos(0);
        robot.setArmPos(250);
        robot.setExtendPos(0.25);
        robot.setArmPos(25);
    }
}