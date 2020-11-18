//package org.firstinspires.ftc.teamcode.Autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Autos.Odometry.EncoderPID;
//import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
//
//@Autonomous(group = "AutoPIDTest", name = "EncoderDriveTest")
//public class EncoderPIDTest extends LinearOpMode {
//    HMap robot = new HMap();
//    EncoderPID drivePID;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        robot.runtime.reset();
//        drivePID = new EncoderPID(2.65, 0.0000005, 00000000, robot,
//                0.0508, .19, 1120, 0.6);
//        robot.resetEncoders();
//
//        waitForStart();
//        drive(-0.40);
//        drive(0.40);
//        turnLeft(95);
//
//        telemetry.addData("Left Response: ", drivePID.response);
//        telemetry.update();
//    }
//
//    private void drive(double distance){
//        drivePID.resetParam();
//        sleep(500);
//        while ((drivePID.success_counter < 50) && (opModeIsActive() && !isStopRequested())) {
//            drivePID.move(distance);
//        }
//    }
//
//    private void drive_left(double distance){
//        drivePID.move_left_to_distance(distance);
//    }
//
//    private void drive_right(double distance){
//        drivePID.move_right_to_distance(distance);
//    }
//
//    public void turnLeft(double angle_of_turn){
//        double turn_distance = drivePID.turn_distance(angle_of_turn);
//        drivePID.resetParam();
//        sleep(500);
//        while ((drivePID.success_counter < 50) && (opModeIsActive() && !isStopRequested())) {
//            drive_left(turn_distance);
//            drive_right(-turn_distance);
//        }
//    }
//}
