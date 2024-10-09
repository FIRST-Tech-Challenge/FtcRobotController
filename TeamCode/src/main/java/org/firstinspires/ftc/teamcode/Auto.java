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

        robot.encoderDrive(1,-15,-15,30);
        robot.encoderStrafe(1, 27,30);
        robot.encoderTurn(1,180,30);

    }
}