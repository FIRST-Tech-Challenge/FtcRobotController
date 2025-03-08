//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.bots.DifferentialWristBot;
//import org.firstinspires.ftc.teamcode.bots.FourWheelDriveBot;
//
//@TeleOp(name = "PodTest")
//public class PodTest extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        DifferentialWristBot robot = new DifferentialWristBot(this);
//
//        robot.isAuto = false;
//        robot.init(hardwareMap);
//        waitForStart();
//        ElapsedTime test = new ElapsedTime();
//        while (opModeIsActive()) {
//
//            telemetry.addData("strafe encoder pos", robot.rightRear.getCurrentPosition());
//            telemetry.addData("vertical right ", robot.rightFront.getCurrentPosition());
//            telemetry.addData("time",test.time());
//            telemetry.update();
////            robot.setLeftDifferentialWristServo(0.3);
////            robot.setRightDifferentialWristServo(0.7);
//        }
//
//
//    }
//}
