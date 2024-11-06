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
        robot.encoderDrive(1,15);
        robot.setArmPos(850);
        robot.setExtendPos(12.5);
        robot.setArmPos(450);
        robot.runIntakeForTime(2.0, -1);
        robot.setExtendPos(0);
        robot.setArmPos(0);

        //puts samples in
        robot.encoderStrafe(1, -26.5);
        robot.encoderTurn(1,-180);
        robot.encoderDrive(1,-36);
        robot.encoderStrafe(1, 10.0);
        robot.encoderDrive(1,46.5);
        robot.encoderDrive(1,-46.5);
        robot.encoderStrafe(1, 9.5);
        robot.encoderDrive(1,46);
        robot.encoderDrive(1,-46);
        robot.encoderStrafe(1, 4);
        robot.encoderDrive(1,46.1);
        robot.setExtendPos(0);
        robot.setArmPos(0);
        //robot.encoderDrive(1,-25);
        //robot.encoderDrive(1,-45);
    }
}