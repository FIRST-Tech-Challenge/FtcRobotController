package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "MainAutonomous")
public class MainAuto extends LinearOpMode{

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //need to clarify values later
    double x,y;
    double randomDistance;

    Bot bot;

    enum Side {
        RIGHT, LEFT, NULL;
    }
    enum DistanceToBackdrop{
        CLOSE, FAR, NULL;
    }

    Side side = Side.NULL;
    DistanceToBackdrop dtb= DistanceToBackdrop.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


        while (!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                side = Side.RIGHT;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                side = Side.LEFT;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                dtb= DistanceToBackdrop.CLOSE;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
                dtb= DistanceToBackdrop.FAR;
            }


            //moves to specified coordinates while maintaining current heading
            Trajectory forward = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(x, y))
                    .build();

            Trajectory strafeLeft = drive.trajectoryBuilder(forward.end())
                    .strafeLeft(randomDistance)
                    .build();

            Trajectory strafeRight = drive.trajectoryBuilder(strafeLeft.end())
                    .strafeRight(randomDistance)
                    .build();


            Trajectory blueAllianceFarNoSense = drive.trajectoryBuilder(strafeLeft.end())
                    .splineTo(new Vector2d(-36,-36), Math.toRadians(0))

                    //spline will occur slower
                    .splineTo(new Vector2d(-36,48), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(0.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                    .splineTo(new Vector2d(-48,48), Math.toRadians(90))
                    .build();

            Thread blueAllianceFarNoSenseThread = new Thread(() -> drive.followTrajectory(blueAllianceFarNoSense));
            //this thread will enable the robot to move forward while performing other activities

            waitForStart();
            if (!isStopRequested()) {

                if(dtb== DistanceToBackdrop.FAR && side==Side.LEFT){
                    blueAllianceFarNoSenseThread.start();
                    //place purple pixel
                    sleep(1000);
                    Bot.fourbar.outtake();
                    Bot.slides.runTo(1);
                    Bot.box.depositFirstPixel();
                    Bot.slides.runTo(2);
                    Bot.box.depositSecondPixel();
                    Bot.resetOuttake();
                    blueAllianceFarNoSenseThread.interrupt();

                }



            }


        }
    }
}
