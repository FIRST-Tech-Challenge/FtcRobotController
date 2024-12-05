package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.network.WifiUtil;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.SpecimanGrabber;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


@Autonomous(name="Example Red Right RR")
public class RedRightExampleRR extends LinearOpMode {

    private MecanumDrive drive = null;
    private Telemetry.Item debugOutout = null;

    private Slide clawSlide = new Slide("lift", Slide.ExtendMotorDirection.Reverse, 2600, 1.0,68.568);
    private SpecimanGrabber specimanGrabber = new SpecimanGrabber();

    public class OpenGrabber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            specimanGrabber.Open();
            return false;
        }
    }
    public class CloseGrabber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            specimanGrabber.Close();
            return false;
        }
    }
    public class LiftToTopBar implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(clawSlide.GetExtendedInches() == 20) { // 18.25
                return true;
            }
            else {
                clawSlide.MoveTo(20, 1.0); //18.25
                return false;
            }
        }
    }
    public class LiftToBottom implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(clawSlide.GetExtendedInches() < 0.5) {
                return true;
            }
            else {
                clawSlide.MoveTo(0, 1.0);
                return false;
            }
        }
    }

    public class LiftToHookPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(clawSlide.GetExtendedInches() == 14.75) {
                return true;
            }
            else {
                clawSlide.MoveTo(15.0, 1.0);
                return false;
            }
        }
    }
    @Override
    public void runOpMode()  throws InterruptedException
    {
        // run once when init is pressed
        drive = new MecanumDrive(this.hardwareMap, new Pose2d(-63, 2, 0));
        clawSlide.Init(hardwareMap);
        specimanGrabber.Init(hardwareMap);

        telemetry.clearAll();
        telemetry.setAutoClear(false);
        //debugOutout = telemetry.addData("Debug:", imu.getRobotYawPitchRollAngles().toString());

        // Delcare Trajectory as such
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .stopAndAdd(new LiftToTopBar()) // lift to be ready to hang
                .lineToX(-31.625) // move forward
                .stopAndAdd(new LiftToHookPosition()) // lower the lift to hang
                .waitSeconds(.250)
                .stopAndAdd(new OpenGrabber()) //open claw
                .lineToX(-41.5) // backup
                .strafeTo(new Vector2d(-41.5, -35)) // strafe right
                .turn(Math.toRadians(180)) // turn 180 degrees
                .lineToX(-15) // drive backward
                .strafeTo(new Vector2d(-15, -50)) // strafe left
                .stopAndAdd(new LiftToBottom()) // lower the lift to the  bottom
                .strafeTo(new Vector2d(-54, -50)) // move foward toward the wall


                //next step
                .lineToX(-15) // drive backward
                .strafeTo(new Vector2d(-15, -58)) // strafe left
                .strafeTo(new Vector2d(-54, -58)) // move foward toward the wall
                .strafeTo(new Vector2d(-54, -40)) // move right to pickup specimen
                .strafeTo(new Vector2d(-65, -40))
                .stopAndAdd(new CloseGrabber())
                .waitSeconds(.250)
                .stopAndAdd(new LiftToTopBar())
                .splineTo(new Vector2d(-31.625, -5), 0)
                .lineToX(-32)

 //               .lineToX(-63)
 //               .stopAndAdd(new CloseGrabber())
 //               .stopAndAdd(new LiftToTopBar())
//                .lineToX(0)
                .build();

        Action TrajectoryAction2 = drive.actionBuilder(drive.pose)
                .stopAndAdd(
                        new LiftToTopBar()
                )
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new CloseGrabber()
                )
        );

        // After we are done initializing our code, we wait for Start button.
        waitForStart();

        // Start button pressed, off we go.

        //go to the bar and hook specimen one
        Actions.runBlocking(TrajectoryAction1);


    }
}
