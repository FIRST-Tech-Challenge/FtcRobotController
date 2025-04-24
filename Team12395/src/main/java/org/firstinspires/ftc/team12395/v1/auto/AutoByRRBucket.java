package org.firstinspires.ftc.team12395.v1.auto;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.team12395.v1.MecanumDrive;

@Config
@Autonomous(name =  "RoadRunner Bucket Auto", group = "Robot")

public class AutoByRRBucket extends LinearOpMode {
    //RobotHardware robot = new RobotHardware(this);
    public class Slides {
        private DcMotor slideMotorLRR;
        private DcMotor slideMotorRRR;

        public Slides(HardwareMap hardwareMap) {
            slideMotorLRR = hardwareMap.get(DcMotorEx.class, "slideMotorL");
            slideMotorLRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotorLRR.setDirection(DcMotor.Direction.FORWARD);

            slideMotorRRR = hardwareMap.get(DcMotorEx.class, "slideMotorR");
            slideMotorRRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotorRRR.setDirection(DcMotor.Direction.REVERSE);
        }

        public class slidesUp implements Action {
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slideMotorLRR.setPower(1);
                    slideMotorRRR.setPower(1);
                    initialized = true;
                }

                // checks lift's current position
                double posL = slideMotorLRR.getCurrentPosition();
                double posR = slideMotorRRR.getCurrentPosition();
                packet.put("slidePosL", posL);
                packet.put("slidePosR", posR);
                if (posL < 3100 || posR < 3100) { // < "encoder ticks"
                    // true causes the action to rerun
                    return true;
                } else {
                    slideMotorLRR.setPower(0);
                    slideMotorRRR.setPower(0);
                    // false stops action rerun
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // said encoder ticks, then powers it off
            }
        }

        public Action slidesUp() {
            return new slidesUp();
        }

        public class slidesDown implements Action {
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slideMotorLRR.setPower(-0.8);
                    slideMotorRRR.setPower(-0.8);
                    initialized = true;
                }

                // checks lift's current position
                double posL = slideMotorLRR.getCurrentPosition();
                double posR = slideMotorRRR.getCurrentPosition();
                packet.put("slidePosL", posL);
                packet.put("slidePosR", posR);
                if (posL > 0 || posR > 0) { // < "encoder ticks"
                    // true causes the action to rerun
                    return true;
                } else {
                    slideMotorLRR.setPower(0);
                    slideMotorRRR.setPower(0);
                    // false stops action rerun
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // the encoder ticks, then powers it off
            }
        }

        public Action slidesDown() {
            return new slidesDown();
        }
    }

    private class OutTake {
        private Servo leftOutTake;
        private Servo rightOutTake;

        public OutTake(HardwareMap hardwareMap) {
            leftOutTake = hardwareMap.get(Servo.class, "leftOutTake");
            rightOutTake = hardwareMap.get(Servo.class, "rightOutTake");
        }

        public class OutTakeHangFirst implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutTake.setPosition(0.67);
                rightOutTake.setPosition(0.33);

                return false;
            }
        }

        public Action outTakeHangFirst() {
            return new OutTakeHangFirst();
        }

        public class OutTakeBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutTake.setPosition(0.75);
                rightOutTake.setPosition(0.25);

                return false;
            }
        }

        public Action outTakeBucket() {
            return new OutTakeBucket();
        }


        public class OutTakeTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftOutTake.setPosition(0.25);
                rightOutTake.setPosition(0.75);

                return false;
            }
        }

        public Action outTakeTransfer() {
            return new OutTakeTransfer();
        }

    }

    public class OutTakeClaw {
        private Servo outClaw;

        public OutTakeClaw(HardwareMap hardwareMap) {
            outClaw = hardwareMap.get(Servo.class, "outClaw");
        }

        public class OutClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outClaw.setPosition(0.75);

                return false;
            }
        }

        public Action outClawOpen() {
            return new OutClawOpen();
        }

        public class OutClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outClaw.setPosition(1);

                return false;
            }
        }

        public Action outClawClose() {
            return new OutClawClose();
        }
    }

    public class Extension {
        private Servo lextend;
        private Servo rextend;

        public Extension(HardwareMap hardwareMap) {
            lextend = hardwareMap.get(Servo.class, "lextend");
            rextend = hardwareMap.get(Servo.class, "rextend");
        }

        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lextend.setPosition(0.8);
                rextend.setPosition(0.215);

                return false;
            }
        }

        public Action extend() {
            return new Extend();
        }

        public class Retract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lextend.setPosition(1);
                rextend.setPosition(0.015);

                return false;
            }
        }

        public Action retract() {
            return new Retract();
        }

    }

    public class Horizontal {
        private Servo horizontal1;

        public Horizontal(HardwareMap hardwareMap) {
            horizontal1 = hardwareMap.get(Servo.class, "horizontal1");
        }

        public class HorizontalUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                horizontal1.setPosition(0.2325);

                return false;
            }
        }

        public Action horizontalUp() {
            return new HorizontalUp();
        }

        public class HorizontalDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                horizontal1.setPosition(0.7585);

                return false;
            }
        }

        public Action horizontalDown() {
            return new HorizontalDown();
        }

    }

    public class IntakeClaw {
        private Servo inClaw;

        public IntakeClaw(HardwareMap hardwareMap) {
            inClaw = hardwareMap.get(Servo.class, "inClaw");
        }

        public class InClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inClaw.setPosition(0.4);

                return false;
            }
        }

        public Action inClawClose() {
            return new InClawClose();
        }

        public class InClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inClaw.setPosition(0);

                return false;
            }
        }

        public Action inClawOpen() {
            return new InClawOpen();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-32.9, -61.25, 0);
        Pose2d Pose2 = new Pose2d(-59.5, -57.5, 45);
        Pose2d Pose3 = new Pose2d(-47, -37, Math.PI / 2);
        Pose2d Pose5 = new Pose2d(-66, -49.5, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Slides slides = new Slides(hardwareMap);
        OutTake outTake = new OutTake(hardwareMap);
        OutTakeClaw outTakeClaw = new OutTakeClaw((hardwareMap));
        Extension extension = new Extension(hardwareMap);
        Horizontal horizontal = new Horizontal(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);


        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(-45)
                .splineToLinearHeading(new Pose2d(-59.5, -57.5,45), 45)
                .build();

        Action tab2 = drive.actionBuilder(Pose2)
                .splineToLinearHeading(new Pose2d(-46, -37,Math.PI/2), 45)
                        .build();

        Action tab3 = drive.actionBuilder(Pose3)
                .splineToLinearHeading(new Pose2d(-61, -60,45), 45)
                        .build();

        Action tab4 = drive.actionBuilder(Pose2)
                .splineToLinearHeading(new Pose2d(-65, -39.5,Math.PI/2), 45)
                .build();
        Action tab5 = drive.actionBuilder(Pose5)
                .splineToLinearHeading(new Pose2d(-63, -65,45), 45)
                .build();

        Action tab6 = drive.actionBuilder(Pose2)
                .splineToLinearHeading(new Pose2d(-38, -20,Math.PI/2), 45)
                        .build();


        waitForStart();

        if (isStopRequested()) return;





        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                extension.retract(),
                                outTakeClaw.outClawClose(),
                                tab1,
                                slides.slidesUp(),
                                outTake.outTakeBucket(),
                                horizontal.horizontalUp()
                        ),
                        sleepAction(100),
                        outTakeClaw.outClawOpen(),
                        sleepAction(200),
                        new ParallelAction(
                                outTake.outTakeTransfer(),
                                slides.slidesDown()
                        ),
                        tab2,
                        horizontal.horizontalDown(),
                        sleepAction(750),
                        intakeClaw.inClawClose(),
                        sleepAction(500),
                        horizontal.horizontalUp(),
                        sleepAction(700),
                        outTakeClaw.outClawClose(),
                        sleepAction(200),
                        new ParallelAction(
                                intakeClaw.inClawOpen(),
                                slides.slidesUp(),
                                tab3
                        ),
                        outTake.outTakeBucket(),
                        sleepAction(550),
                        outTakeClaw.outClawOpen(),
                        sleepAction(200),
                        new ParallelAction(
                                tab4,
                                outTake.outTakeTransfer(),
                                horizontal.horizontalDown(),
                                slides.slidesDown()
                        ),
                        sleepAction(500),
                        intakeClaw.inClawClose(),
                        sleepAction(500),
                        horizontal.horizontalUp(),
                        sleepAction(750),
                        outTakeClaw.outClawClose(),
                        sleepAction(200),
                        new ParallelAction(
                                intakeClaw.inClawOpen(),
                                slides.slidesUp(),
                                tab5
                        ),
                        outTake.outTakeBucket(),
                        sleepAction(500),
                        outTakeClaw.outClawOpen(),
                        sleepAction(500),
                        outTake.outTakeTransfer(),
                        slides.slidesDown(),
                        tab6
                )
        );
    }

    private Action sleepAction(long milliseconds) {
        return (TelemetryPacket packet) -> {
            sleep(milliseconds);
            return false;
        };
    }
}