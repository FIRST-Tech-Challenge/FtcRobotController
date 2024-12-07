package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.SpecimanGrabber;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


@Autonomous(name="3 specimen auto")
public class SpecimenAuto3 extends LinearOpMode {

    private MecanumDrive drive = null;
    private Telemetry.Item debugOutout = null;

    private Slide clawSlide = new Slide("lift", "resetlift", Slide.ExtendMotorDirection.Reverse, 2600, 1.0,68.568);
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
            if(clawSlide.GetExtendedInches() < 0.1) {
                return true;
            }
            else {
                clawSlide.MoveTo(0, 1.0);
                clawSlide.ProcessLoop();
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
        drive = new MecanumDrive(this.hardwareMap, new Pose2d(-62.175, 2, 0));
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
                .strafeTo(new Vector2d(-41.5, -36)) // strafe right
                .turn(Math.toRadians(180)) // turn 180 degrees
                .lineToX(-15) // drive backward
                .strafeTo(new Vector2d(-15, -50)) // strafe left
                .stopAndAdd(new LiftToBottom()) // lower the lift to the  bottom
                .strafeTo(new Vector2d(-54, -50)) // move foward toward the wall


                //next step
                .strafeTo(new Vector2d(-54, -40)) // move right to pickup specimen
                .strafeTo(new Vector2d(-65, -40))
                .stopAndAdd(new CloseGrabber())
                .waitSeconds(.250)
                .stopAndAdd(new LiftToTopBar())
                .splineTo(new Vector2d(-31.625, -5), 0)
                .stopAndAdd(new LiftToHookPosition())
                .waitSeconds(.250)
                .stopAndAdd(new OpenGrabber())
                .waitSeconds(.250)
                .stopAndAdd(new LiftToBottom())
                .lineToX(-35)
                .splineTo(new Vector2d(-54, -40),0)
                .strafeTo(new Vector2d(-65, -40))
                .stopAndAdd(new CloseGrabber())
                .waitSeconds(.150)
                .stopAndAdd(new LiftToTopBar())
                .splineTo(new Vector2d(-31.625, -7), 0)
                .stopAndAdd(new LiftToHookPosition())
                .waitSeconds(.250)
                .stopAndAdd(new OpenGrabber())
                .lineToX(-35)
                .waitSeconds(.250)
                .stopAndAdd(new LiftToBottom())

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
