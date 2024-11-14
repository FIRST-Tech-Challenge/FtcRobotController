package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.PIDFMotorController;

@Config
@Autonomous
public class autoRoadRunnerTest extends LinearOpMode {
    public static boolean USER_INPUT_FLAG = false;
    public static class Slides {
        private final PIDFMotorController slideController;

        public Slides(HardwareMap hardwareMap) {
            DcMotorEx rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double SLIDE_TICKS_IN_DEGREES = 537.7 / 360.0;
            slideController = new PIDFMotorController(rightSlideMotor, 0.01, 0.25, 0.001, 0, SLIDE_TICKS_IN_DEGREES, 0.5, 2);
        }

        public class SlidesUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.driveToPosition(1900);
                return false;
            }
        }

        public Action slidesUp() {
            return new SlidesUp();
        }

        public class SlidesDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.driveToPosition(0);
                return false;
            }
        }

        public Action slidesDown() {
            return new SlidesDown();
        }

        public class SlidesHold implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.driveToPosition(1270);
                return false;
            }
        }

        public Action slidesHold() {
            return new SlidesHold();
        }
    }
    @Config
    public class WaitForUser implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            while(!USER_INPUT_FLAG) {
                sleep(100);
            }
            USER_INPUT_FLAG = false;
            return false;
        }
    }

    public Action waitForUser() {
        return new WaitForUser();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(13, -61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Slides slides = new Slides(hardwareMap);

        TrajectoryActionBuilder firstSpec0 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(13,-50))
                .waitSeconds(3);

        TrajectoryActionBuilder firstSpec = drive.actionBuilder(beginPose)
                .waitSeconds(2)// go to sub, place spec
                .strafeTo(new Vector2d(0, -30))
                .waitSeconds(2);

        Action trajectoryAction0 = firstSpec0.build();
        Action trajectoryAction1 = firstSpec.build();

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction0,
                        waitForUser(),
                        slides.slidesUp(),
                        trajectoryAction1,
                        slides.slidesHold(),
                        trajectoryAction0
                )
        );
    }
}