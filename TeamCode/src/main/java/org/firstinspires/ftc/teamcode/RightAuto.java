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

        //drives forward and puts the specimen on the rung
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
        robot.encoderStrafe(1, 15);
        robot.encoderDrive(1,46.1);
        robot.encoderDrive(1,-15);
        sleep(800);
        robot.runIntakeForTime(1.5, -1);
        robot.encoderDrive(1, 15.89);
        robot.setExtendPos(0.25);
        robot.setArmPos(315);
        robot.setExtendPos(5.0);
        robot.runIntakeForTime(.75, 1);
        robot.setArmPos(580);
        robot.encoderDrive(1, -6.9);
        robot.encoderStrafe(1, -70);
        robot.encoderTurn(1, 180);
        robot.encoderDrive(1,9.8898);
        robot.setArmPos(1051);
        robot.setExtendPos(12.001);
        robot.setArmPos(450);
        robot.setExtendPos(0.25);
        robot.setArmPos(25);
        //robot.encoderDrive(1,-25);
        //robot.encoderDrive(1,-45);
    }
}