//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Old.Robots.FWDRobot;
//import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
//
//
//@TeleOp(name = "MLOTeleop")
//public class MLOTeleop extends LinearOpMode {
//
//    public void runOpMode() {
//
//        telemetry.addData("Status", "Before new Robot");
//        telemetry.update();
//        FWDRobot robot = new FWDRobot(this, true);
//        telemetry.addData("Status", "Done with new Robot");
//        telemetry.update();
//        //robot.navigateTeleOp();
//        double magnitude;
//        double angleInRadian;
//        double angleInDegree;
//        boolean slowMode = false;
//
//        telemetry.addData("Status", "Ready to go");
//        telemetry.update();
//
//        //Aiden - during competition day robot disconnected so we are trying this code
//        while (!opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("status", "waiting for start command...");
//            telemetry.update();
//        }
//
//
//        while (!isStopRequested()) {
//            //ignore cuz we using two gamepad
//            double left_stick_x = gamepad1.left_stick_x;
//            double right_stick_x = gamepad1.right_stick_x;
//            double right_stick_y = gamepad1.right_stick_y;
//
//            angleInRadian = Math.atan2(left_stick_y_2, left_stick_x_2*-2);
//
//            angleInDegree = Math.toDegrees(angleInRadian);
//
//            magnitude = Math.sqrt(Math.pow(left_stick_x_2, 2) + Math.sqrt(Math.pow(left_stick_y_2, 2)));
//            robot.moveMultidirectional(left_stick_y_2, 0.95, (float)(right_stick_x_2*0.6), slowMode); // It is 0.95, because the robot DCs at full power.
//        }
//        idle();
//    }
//}