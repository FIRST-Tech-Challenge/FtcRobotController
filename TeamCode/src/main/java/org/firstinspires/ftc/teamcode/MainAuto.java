package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "MainAutonomous")
public class MainAuto extends LinearOpMode{

    //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Bot bot;

    enum Side {
        RIGHT, LEFT, NULL;
    }

    Side side = Side.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);


        while (!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                side = Side.RIGHT;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                side = Side.LEFT;
            }


        /*    Trajectory forward = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(52, moveDiff))
                    .build();

            Trajectory parkLeft = drive.trajectoryBuilder(forward.end())
                    .strafeLeft(22)
                    .build();
            Trajectory parkRight = drive.trajectoryBuilder(forward.end())
                    .strafeRight(24)
                    .build();

         */






        }
    }
}
