//package org.firstinspires.ftc.teamcode.others;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//
//@TeleOp
//public class BasicArcadeDriveOpmode extends OpMode {
//
//    @Override
//    public void init() {
//
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void start() {
//        telemetry.addData("Status", "OpMode is starting");
//    }
//
//
//    @Override
//    public void loop() {
////        double drive = -gamepad1.left_stick_y;
////        double turn = gamepad1.left_stick_x;
//
////        double leftPower = drive + turn;
////        double rightPower = drive - turn;
//
////        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
////        if (max > 1.0) {
////            leftPower = leftPower / max;
////            rightPower = rightPower / max;
////        }
////
////        left_motor.setPower(leftPower);
////        right_motor.setPower(rightPower);
////
////        telemetry.addData("Status", "OpMode is looping");
////        telemetry.addData("Left Drive Power", leftPower);
////        telemetry.addData("Right Drive Power", rightPower);
//
////        if (gamepad1.a) {
////            test_servo.setPosition(0.5);
////        }
//
////    }
////}
