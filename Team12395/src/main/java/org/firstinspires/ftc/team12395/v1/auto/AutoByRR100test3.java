package org.firstinspires.ftc.team12395.v1.auto;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.team12395.v1.MecanumDrive;

@Config
@Disabled
@Autonomous(name =  "Auto By RoadRunner 100pts test3", group = "Robot")

public class AutoByRR100test3 extends LinearOpMode{
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
                if (posL < 1087 || posR < 1087) { // < "encoder ticks"
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
        private class OutTake  {
            private Servo leftOutTake;
            private Servo rightOutTake;

            public OutTake(HardwareMap hardwareMap){
                leftOutTake = hardwareMap.get(Servo.class, "leftOutTake");
                rightOutTake = hardwareMap.get(Servo.class, "rightOutTake");
            }
            public class OutTakeHang implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftOutTake.setPosition(0.67);
                    rightOutTake.setPosition(0.33);

                    return false;
                }
            }
            public Action outTakeHang(){
                return new OutTakeHang();
            }
            public class OutTakeTransfer implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftOutTake.setPosition(0.25);
                    rightOutTake.setPosition(0.75);

                    return false;
                }
            }
            public Action outTakeTransfer(){
                return new OutTakeTransfer();
            }

        }

        public class OutTakeClaw {
            private Servo outClaw;

            public OutTakeClaw(HardwareMap hardwareMap){
                outClaw = hardwareMap.get(Servo.class, "outClaw");
            }

            public class OutClawOpen implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    outClaw.setPosition(0.45);

                    return  false;
                }
            }
            public Action outClawOpen(){
                return new OutClawOpen();
            }

            public class OutClawClose implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    outClaw.setPosition(0.75);

                    return false;
                }
            }
            public Action outClawClose(){
                return new OutClawClose();
            }
        }

        public class Extension {
            private Servo lextend;
            private Servo rextend;

            public Extension(HardwareMap hardwareMap){
                lextend = hardwareMap.get(Servo.class, "lextend");
                rextend = hardwareMap.get(Servo.class, "rextend");
            }

            public class Extend implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    lextend.setPosition(1);
                    rextend.setPosition(0.015);

                    return false;
                }
            }

            public Action extend(){
                return new Extend();
            }

            public class Retract implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    lextend.setPosition(0.8);
                    rextend.setPosition(0.215);

                    return false;
                }
            }

            public Action retract(){
                return new Retract();
            }

        }

        public class Horizontal{
            private Servo horizontal1;

            public Horizontal(HardwareMap hardwareMap){
                horizontal1 = hardwareMap.get(Servo.class, "horizontal1");
            }
            public class HorizontalUp implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    horizontal1.setPosition(0.25);

                    return false;
                }
            }

            public Action horizontalUp(){
                return new HorizontalUp();
            }

            public class HorizontalDown implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    horizontal1.setPosition(0.75);

                    return false;
                }
            }

            public Action horizontalDown(){
                return new HorizontalDown();
            }

        }

        public class IntakeClaw{
            private Servo inClaw;

            public IntakeClaw(HardwareMap hardwareMap){
                inClaw = hardwareMap.get(Servo.class, "inClaw");
            }
            public class InClawClose implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    inClaw.setPosition(0.4);

                    return false;
                }
            }
            public Action inClawClose(){
                return new InClawClose();
            }

            public class InClawOpen implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    inClaw.setPosition(0);

                    return false;
                }
            }

            public Action inClawOpen() {
                return new InClawOpen();
            }
        }

    @Override
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(0, 0, -Math.PI/2);
        Pose2d SecondPose = new Pose2d(9.5, -31, -Math.PI/2);
        Pose2d ThirdPose = new Pose2d(64, -62, -Math.PI/2);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //robot.init();

        Slides slides = new Slides(hardwareMap);
        OutTake outTake = new OutTake(hardwareMap);
        OutTakeClaw outTakeClaw = new OutTakeClaw((hardwareMap));
        Extension extension = new Extension(hardwareMap);
        Horizontal horizontal = new Horizontal(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);

        TrajectoryActionBuilder waitHalf = drive.actionBuilder(initialPose)
                .waitSeconds(.5);
        TrajectoryActionBuilder waitOne = drive.actionBuilder(initialPose)
                .waitSeconds(1);



        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-10)
                .waitSeconds(5);





        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .build();

        waitForStart();




        if (isStopRequested()) return;



        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();


        Action wait05;
        wait05 = waitHalf.build();

        Action wait1;
        wait1 = waitOne.build();


        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,

                        trajectoryActionCloseOut
                )
        );
    }
}