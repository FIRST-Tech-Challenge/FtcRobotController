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

        //drives forward and puts the specimen on the hook
        robot.encoderDrive(1,15);
        robot.setArmPos(750);
        robot.setExtendPos(12.75);
        robot.setArmPos(450);
        robot.setExtendPos(0);
        robot.setArmPos(0);
        robot.encoderStrafe(1, -33);
        robot.encoderTurn(1,-180);
        robot.encoderDrive(1,-36);
        robot.encoderStrafe(1, 11.3);
        robot.encoderDrive(1,45);
        robot.encoderDrive(1,-45);
        robot.encoderStrafe(1, 11.35);
        robot.encoderDrive(1,46);
        robot.encoderDrive(1,-46);
        robot.encoderStrafe(1, 11.05);
        robot.encoderDrive(1,46.5);
        robot.encoderDrive(1,-25);
        //robot.encoderDrive(1,-45);
    }
}