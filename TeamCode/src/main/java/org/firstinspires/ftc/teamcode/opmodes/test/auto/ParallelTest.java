//package org.firstinspires.ftc.teamcode.opmodes.test.auto;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;
////import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Arm;
////import org.firstinspires.ftc.teamcode.opmodes.auto.presets.LinearSlide;
//
//@Autonomous(name = "Arm+Slide Auto test", group = "Auto")
//public class ParallelTest extends LinearOpMode {
//    private MecanumDrive drive;
//  //  private Arm arm;
//   // private LinearSlide slide;
//
//    @Override
//    public void runOpMode() {
//        // Initialize subsystems
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); // Starting position
//        //arm = new Arm(hardwareMap);
//        //slide = new LinearSlide(hardwareMap);
//        linearSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "armMotor");
//        Servo rollServo = hardwareMap.get(Servo.class, "roll");
//        Servo yawServo = hardwareMap.get(Servo.class, "yaw");
//        Servo pitchServo = hardwareMap.get(Servo.class, "pitch");
//        Servo clawServo = hardwareMap.get(Servo.class, "claw");
//        DcMotor armMotor = hardwareMap.dcMotor.get("liftMotor");
//
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // Run the sequence
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(0, 0, 0)) // Starting pose
//                        .strafeTo(new Vector2d(20, 0))  // 1. Strafe forward 20 inches
//                        .waitSeconds(0.5)
//
//                        // 2. Lift the arm
////                        .stopAndAdd(new ArmSpecimen(arm, 30));
//                        .afterTime(0, arm.raiseArmForUpperBasket())
//                        .waitSeconds(0.5)
//
//                        // 3. Extend the linear slide
//                        .afterTime(0, slide.extendSlideForPickFromPool())
//                        .waitSeconds(0.5)
//
//                        // 4. Lower the arm slightly
//                        .afterTime(0, arm.raiseArmForSpecimenPickUpFromWall())
//                        .waitSeconds(0.3)
//
//                        // 5. Raise the arm back to its original position
//                        .afterTime(0, arm.raiseArmForUpperBasket())
//                        .waitSeconds(0.3)
//
//                        // 6. Retract the linear slide
//                        .afterTime(0, slide.retractSlideBackward())
//                        .waitSeconds(0.5)
//
//                        .build()
//        );
//
//    }
//
//    public class ArmSpecimen implements Action{
//        DcMotor arm;
//        double position;
//
//        public ArmSpecimen(DcMotor arm, double p){
//            this.arm = arm;
//            this.position = p;
//        }
//        @Override
//        public boolean run(@NonNull TelemeteryPacket telemeteryPacket){
//            arm.setTargetPosition(position);
//            return false;
//        }
//    }
//}
