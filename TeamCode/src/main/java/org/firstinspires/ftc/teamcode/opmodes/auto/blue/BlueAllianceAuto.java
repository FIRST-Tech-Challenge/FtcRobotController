//package org.firstinspires.ftc.teamcode.opmodes.auto.blue;
//
//import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
//
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Arm;
//import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Claw;
//import org.firstinspires.ftc.teamcode.opmodes.auto.presets.LinearSlide;
//import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Roll;
//import org.firstinspires.ftc.teamcode.opmodes.auto.presets.XYaw;
//import org.firstinspires.ftc.teamcode.opmodes.auto.presets.YPitch;
//import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;
//
//@Autonomous(name="Blue Auto", group = "Blue Alliance", preselectTeleOp = "RobotCentricDrive")
//public class BlueAllianceAuto extends LinearOpMode {
//
//    private final Servo claw;
//    private final DcMotorEx linearSlideMotor;
//    private final Servo pitch;
//    private final Servo yaw;
//    private final Servo roll;
//    private static DcMotorEx armMotor = null;
//
//
//    public static String TEAM_NAME = "Pack-A-Punch";
//    public static int TEAM_NUMBER = 26396;
//
//    public BlueAllianceAuto(Servo claw, DcMotorEx linearSlideMotor, Servo pitch, Servo yaw, Servo roll, DcMotorEx armMotor) {
//        this.claw = claw;
//        this.linearSlideMotor = linearSlideMotor;
//        this.pitch = pitch;
//        this.yaw = yaw;
//        this.roll = roll;
//        this.armMotor = armMotor;
//    }
//
//    public enum START_POSITION{
//        LEFT,
//        RIGHT,
//    }
//    public static START_POSITION startPosition;
//
//        public void runOpMode() {
//            //Key Pad input to selecting Starting Position of robot
//            telemetry.setAutoClear(true);
//            telemetry.clearAll();
//            while(!isStopRequested()){
//                telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
//                        TEAM_NAME, " ", TEAM_NUMBER);
//                telemetry.addData("---------------------------------------","");
//                telemetry.addData("Select Starting Position  on gamepad 1:","");
//                telemetry.addData("    Left   ", " = X ");
//                telemetry.addData("    Right ", " = Y ");
//                if(gamepad1.x){
//                    startPosition = START_POSITION.LEFT;
//                    break;
//                }
//                if(gamepad1.y){
//                    startPosition = START_POSITION.RIGHT;
//                    break;
//                }
//                telemetry.update();
//            }
//            telemetry.setAutoClear(false);
//            telemetry.clearAll();
//
//            telemetry.addData("Selected Starting Position", startPosition);
//            telemetry.update();
//
//            waitForStart();
//
//            //Game Play Button  is pressed
//            if (opModeIsActive() && !isStopRequested()) {
//                new Arm.DeactivateArm();
//                //Build parking trajectory based on last detected target by vision
//                runAutonoumousMode();
//            }
//        }
//
//        public void runAutonoumousMode() {
//            Pose2d initPose = new Pose2d(0, 70, Math.toRadians(90)); // Starting Pose
//            Pose2d SpecimenHang = new Pose2d(0, 27, Math.toRadians(-90));
//            Pose2d MoveSample1 = new Pose2d(49, 35, Math.toRadians(-90));
//            Pose2d PickSample1 = new Pose2d(49, 24, Math.toRadians(-90));
//            Pose2d PositionSample1 = new Pose2d(49, 49, Math.toRadians(44));
//            Pose2d Sample1High = new Pose2d(57, 58, 44);
//            Pose2d PickSample2 = new Pose2d(59, 25, -90);
//            Pose2d Sample2High = new Pose2d(61, 49, 62.49);
//            Pose2d PickSample3 = new Pose2d(69, 33, -90);
//            Pose2d Sample3High = new Pose2d(68, 52, 83.38);
//            Pose2d Ascent1 = new Pose2d(27, 5, 180);
//
//            Pose2d initPose1 = new Pose2d(0, 60, 180.00);
//            Pose2d Pose1 = new Pose2d(0, 33, 180.00);
//            Pose2d Pose2 = new Pose2d(-35, 33, 180.00);
//            Pose2d Pose3 = new Pose2d(-35, -5, 90);
//            Pose2d Pose4 = new Pose2d(-44, -5, 90);
//            Pose2d Pose5 = new Pose2d(-44, 40, 90);
//            Pose2d Pose6 = new Pose2d(-44, -5, 90);
//            Pose2d Pose9 = new Pose2d(-55, 6, 90);
//            Pose2d Pose10 = new Pose2d(-55, 40, 90);
//            Pose2d Pose11 = new Pose2d(-55, 6, 90);
//
//            double waitSecondsBeforeDrop = 0;
//            if (startPosition == START_POSITION.LEFT) {
//                MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
//
//
//                //Move Robot to Position to hand a specimen on the high chamber
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .splineTo(SpecimenHang.position, SpecimenHang.heading)
//                                .build());
//                safeWaitSeconds(0.1);
//                telemetry.addLine("Move robot to submersible to place specimen on chamber!");
//                telemetry.update();
//                new ParallelAction(
//                                Arm.raiseArmForLowerBasket(),
//                                LinearSlide.extendArmForward(),
//                                new YPitch.MoveWristUp(),
//                               new XYaw.MoveWristToLeft(),
//                                new Roll.Rotate90Clockwise(),
//                                new Claw.OpenClaw()
//                        );
//                new ParallelAction(
//                        new LinearSlide.RetractArmBackward()
//                );
//                safeWaitSeconds(0.1);
//                telemetry.addLine("Place specimen on chamber!");
//                telemetry.update();
//
//                //Move to pick up Sample 1
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .strafeTo(MoveSample1.position)
//                                .build());
//                safeWaitSeconds(0.1);
//                telemetry.addLine("Move Robot to pick up yellow Sample 1!");
//                telemetry.update();
//
//                //Pick Up Sample 1
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToY(PickSample1.position.y)
//                                .build());
//                safeWaitSeconds(0.1);
//                new Arm.RaiseArmForSamplePickupFromFloor();
//                new Claw.CloseClaw();
//                telemetry.addLine("Pick up yellow Sample 1!");
//                telemetry.update();
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .splineTo(PositionSample1.position, PositionSample1.heading)
//                                .build());
//                safeWaitSeconds(0.1);
//                new Arm.RaiseArmForHighBasket();
//                telemetry.addLine("Move to drop yellow Sample 1!");
//                telemetry.update();
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .splineTo(Sample1High.position, Sample1High.heading)
//                                .build());
//                safeWaitSeconds(0.1);
//                new LinearSlide.ExtendLinearSlide();
//                new Claw.OpenClaw();
//                telemetry.addLine("Drop yellow Sample 1");
//                telemetry.update();
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .splineTo(Ascent1.position, Ascent1.heading)
//                                .build());
//                safeWaitSeconds(0.1);
//                new Arm.RaiseArmForLowerBasket();
//                telemetry.addLine("Level 1 Ascent Completed!");
//                telemetry.addLine("Blue Left Autonomous Complete!");
//            }
//            else {
//                MecanumDrive drive = new MecanumDrive(hardwareMap, initPose1);
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToY(Pose1.position.y)
//                                .build());
//                safeWaitSeconds(0.1);
//                //Telemetry
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToX(Pose2.position.x)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                safeWaitSeconds(0.1);
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToY(Pose3.position.y)
//                                .build());
//                safeWaitSeconds(0.1);
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .strafeTo(Pose4.position)
//                                .build());
//                safeWaitSeconds(0.1);
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToY(Pose5.position.y)
//                                .build());
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToY(Pose6.position.y)
//                                .build());
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .strafeTo(Pose9.position)
//                                .build());
//                safeWaitSeconds(0.1);
//
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToY(Pose10.position.y)
//                                .build());
//                Actions.runBlocking(
//                        drive.actionBuilder(drive.pose)
//                                .lineToY(Pose11.position.y)
//                                .build());
//            }
//        }
//    public void safeWaitSeconds(double time) {
//        ElapsedTime timer = new ElapsedTime(SECONDS);
//        timer.reset();
//        while (!isStopRequested() && timer.time() < time) {
//        }
//    }
//}