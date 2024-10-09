package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="auto", group="auto")
public class Auto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Bot robot = new Bot(this);

        robot.init(hardwareMap);

        waitForStart();

        robot.encoderDrive(1,15);
        robot.encoderStrafe(1, 32);
        robot.encoderTurn(1,180);
        robot.encoderDrive(1,-35);
        robot.encoderStrafe(1, -12);
        robot.encoderDrive(1,45);
        robot.encoderTurn(1,45);
    }
}