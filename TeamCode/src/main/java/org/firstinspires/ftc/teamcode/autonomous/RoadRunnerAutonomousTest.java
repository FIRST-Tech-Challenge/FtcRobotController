package org.firstinspires.ftc.teamcode.autonomous;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class RoadRunnerAutonomousTest extends LinearOpMode {
    public class Slides {
        private DcMotorEx rightSlideMotor;

        public Slides(HardwareMap hardwareMap) {
            rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class rightSlideMotorUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlideMotor.setPower(0.8);
                    initialized = true;
                }

                double pos = rightSlideMotor.getCurrentPosition();
                packet.put("rightSlideMotorPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    rightSlideMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action slidesUp() {
            return new rightSlideMotorUp();
        }

        public class rightSlideMotorDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlideMotor.setPower(-0.8);
                    initialized = true;
                }

                double pos = rightSlideMotor.getCurrentPosition();
                packet.put("rightSlideMotorPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    rightSlideMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action slidesDown(){
            return new rightSlideMotorDown();
        }
    }

    public class Claw {
        private Servo specServo;

        public Claw(HardwareMap hardwareMap) {
            specServo = hardwareMap.get(Servo.class, "specServo");
        }

        public class closeSpecServo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new closeSpecServo();
        }

        public class OpenSpecServo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenSpecServo();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw specServo = new Claw(hardwareMap);
        Slides slides = new Slides(hardwareMap);


        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
        }
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        slides.slidesUp(),
                        specServo.openClaw(),
                        slides.slidesDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
