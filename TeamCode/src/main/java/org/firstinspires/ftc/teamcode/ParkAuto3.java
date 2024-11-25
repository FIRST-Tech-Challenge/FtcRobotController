//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//@Autonomous(name = "ParkAuto", group = "Autonomous")
//
//public class ParkAuto3 extends LinearOpMode {
//    DcMotor FrontLeft;
//    DcMotor FrontRight;
//    DcMotor BackLeft;
//    DcMotor BackRight;
//    DcMotor EncoderWheel;
//
//    int TicksPerRotation = 2000;
//    double EncoderWheelDiameter = 1.82;
//    int StartingPosition;
//
//    Double Lift_power = 0.5;
//    DcMotor LiftLeft;
//    DcMotor LiftRight;
//    Servo ServoDump;
//
//    /* Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//
//        EncoderWheel = hardwareMap.get(DcMotor.class, "EncoderWheel");
//
//        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
//        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
//
//        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
//        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
//
//        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        LiftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
//        LiftRight = hardwareMap.get(DcMotor.class, "LiftRight");
//
//        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        ServoDump = hardwareMap.get(Servo.class, "ServoDump");
//
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//
//
//        StartingPosition = EncoderWheel.getCurrentPosition();
//
//        waitForStart();
//        if (opModeIsActive()) {
//
//            // Move the robot backward
//            MoveBackward(5);  // 48 inches is an example, adjust as needed
//
//            // Move the lift up
//            moveLift(0.25, 3.0);  // 1.0 is the power and 2.0 is the duration (seconds)
//
//            // Move the servo to position
//            moveServo(0.25);  // Set servo to position 0.5 (adjust as needed)
//
//            // Move the servo back to the original position
//            moveServo(1);  // Move servo to position 0 (adjust as needed)
//
//            // Move the lift down
//            moveLift(-0.5, 0.75);  // -1.0 is the power for moving down, 2.0 seconds
//
//            // Move the robot backward more
//            moveRobotBackward(3);  // Move an additional 24 inches (adjust as needed)
//
//        }
//
//
////
////        //go forward
////        FrontLeft.setPower(0.5);
////        FrontRight.setPower(0.5);
////        BackLeft.setPower(0.5);
////        BackRight.setPower(0.5);
////
////        //park
////        while (opModeIsActive()) {
////            double Distance = getDistance();
////            int LiftRightPosition = LiftRight.getCurrentPosition();
////
////            telemetry.addData("Distance: ", Distance);
////            telemetry.addData("LiftRightPosition:", LiftRightPosition);
////
////            if (Distance <= -3.5) {
////                //stop
////                FrontLeft.setPower(0);
////                FrontRight.setPower(0);
////                BackLeft.setPower(0);
////                BackRight.setPower(0);
////
////            if (LiftRightPosition > -1250 && !BackLeft.isBusy() || !BackRight.isBusy() || !FrontLeft.isBusy() || !FrontRight.isBusy()) {
////                LiftLeft.setPower(-Lift_power);
////                LiftRight.setPower(-Lift_power);
////                }
////            if (LiftRightPosition < -2940) {
////                ServoDump.setPosition(0.25);
////                sleep(1000);
////            }
////        }
////        telemetry.update();
////       }
//    }
//
//    private boolean MoveDumpBucketToPosition(Servo servoDump){
//
//        double currentPosition = servoDump.getPosition();
//        servoDump.setPosition(0.25);
//        sleep(1000);
//        servoDump.setPosition(currentPosition);
//
//        return true;
//    }
//
//    private  boolean MoveLiftToMaxPosition(DcMotor LiftLeft, DcMotor LiftRight){
//
//        boolean taskCompleted = false;
//
//        int LiftRightPosition;
//
//        LiftLeft.setPower(-Lift_power);
//        LiftRight.setPower(-Lift_power);
//
//        while (opModeIsActive()) {
//
//            LiftRightPosition = LiftRight.getCurrentPosition();
//            telemetry.addData("LiftRightPosition:", LiftRightPosition);
//
//            if (LiftRightPosition < -1250) {
//
//                taskCompleted = true;
//                break;
//            }
//        }
//
//        LiftLeft.setPower(0);
//        LiftRight.setPower(0);
//
//        return taskCompleted;
//    }
//
//    private boolean MoveToPosition(){
//
//        boolean taskCompleted = false;
//
//        //go forward
//        FrontLeft.setPower(0.5);
//        FrontRight.setPower(0.5);
//        BackLeft.setPower(0.5);
//        BackRight.setPower(0.5);
//
//        while (opModeIsActive()) {
//            double Distance = getDistance();
//
//            telemetry.addData("Distance: ", Distance);
//
//            if (Distance <= -3.5) {
//                taskCompleted = true;
//                break;
//            }
//        }
//
//        //stop
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//
//        return taskCompleted;
//    }
//
//    private double getDistance() {
//        double Rotations = (double) (EncoderWheel.getCurrentPosition() - StartingPosition) / TicksPerRotation;
//        return Rotations * EncoderWheelDiameter * Math.PI;
//    }
//}