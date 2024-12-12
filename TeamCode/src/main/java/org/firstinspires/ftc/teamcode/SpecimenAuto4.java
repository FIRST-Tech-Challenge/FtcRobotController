package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PathBuilder;
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


@Autonomous(name="4 specimen auto")
public class SpecimenAuto4 extends LinearOpMode {

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
                //first hang completed
                .splineTo(new Vector2d(-13.5, -38), Math.toRadians(0)) // move to first push
                .strafeTo(new Vector2d(-13.5, -49)) // move in front of sample
                .strafeTo(new Vector2d(-45, -49)) // pushes
                .strafeTo(new Vector2d(-13.5, -49)) // backup
                .strafeTo(new Vector2d(-13.5, -57)) // move in front of 2nd sample
                .strafeTo(new Vector2d(-45, -57)) // pushes second sample
                .stopAndAdd(new LiftToBottom()) // lower the lift to the  bottom
                //blocks pushed
                .strafeTo(new Vector2d(-45, -38)) // move left in front of specimen on wall
                .strafeTo(new Vector2d(-62,-38)) // move forward to pickup specimen
                .stopAndAdd(new CloseGrabber()) // close claw
                .waitSeconds(.200)
                .stopAndAdd(new LiftToTopBar()) // lift to top
                .strafeToLinearHeading(new Vector2d(-31.625, -5), Math.toRadians(0)) // strafe to submersible
                .stopAndAdd(new LiftToHookPosition()) // lower lift to hook
                .waitSeconds(.200)
                .stopAndAdd(new OpenGrabber()) // open claw
                .stopAndAdd(new LiftToBottom()) // lower lift to bottom
                //second hang completed
                .strafeToLinearHeading(new Vector2d(-62, -40), Math.toRadians(180)) // strafe back to the wall to pick up the specimen
                .waitSeconds(.200)
                .stopAndAdd(new CloseGrabber()) // close the claw
                .waitSeconds(.200)
                .stopAndAdd(new LiftToTopBar()) // lift to top
                .strafeToLinearHeading(new Vector2d(-31.625, -7), Math.toRadians(0)) // strafe back to submersible
                .stopAndAdd(new LiftToHookPosition()) // lower lift to hook
                .waitSeconds(.200)
                .stopAndAdd(new OpenGrabber()) // open claw
                .stopAndAdd(new LiftToBottom()) // lower lift to bottom
                //third hang completed
                .strafeToLinearHeading(new Vector2d(-62, -40), Math.toRadians(180)) // strafe back to the wall to pick up the specimen
                .waitSeconds(.200)
                .stopAndAdd(new CloseGrabber()) // close the claw
                .waitSeconds(.200)
                .stopAndAdd(new LiftToTopBar()) // lift to top
                .strafeToLinearHeading(new Vector2d(-31.625, -9), Math.toRadians(0)) // strafe back to submersible
                .stopAndAdd(new LiftToHookPosition()) // lower lift to hook
                .waitSeconds(.200)
                .stopAndAdd(new OpenGrabber()) // open claw
                .stopAndAdd(new LiftToBottom()) // lower lift to bottom
                .build();
//
//                //old code
//                .strafeTo(new Vector2d(-54, -38)) // move right to pickup specimen
//                .strafeTo(new Vector2d(-65, -40))
//                .stopAndAdd(new CloseGrabber())
//                .waitSeconds(.250)
//                .stopAndAdd(new LiftToTopBar())
//                .splineTo(new Vector2d(-31.625, -5), 0)
//                .stopAndAdd(new LiftToHookPosition())
//                .waitSeconds(.250)
//                .stopAndAdd(new OpenGrabber())
//                .waitSeconds(.250)
//                .stopAndAdd(new LiftToBottom())
//                .lineToX(-35)
//                .splineTo(new Vector2d(-54, -40),0)
//                .strafeTo(new Vector2d(-65, -40))
//                .stopAndAdd(new CloseGrabber())
//                .waitSeconds(.150)
//                .stopAndAdd(new LiftToTopBar())
//                .splineTo(new Vector2d(-31.625, -7), 0)
//                .stopAndAdd(new LiftToHookPosition())
//                .waitSeconds(.250)
//                .stopAndAdd(new OpenGrabber())
//                .lineToX(-35)
//                .waitSeconds(.250)
//                .stopAndAdd(new LiftToBottom())
//                .waitSeconds(1.5)
//                .build();
//
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
