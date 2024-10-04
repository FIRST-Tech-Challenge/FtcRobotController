//package org.firstinspires.ftc.teamcode.RoadRunner.messages;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
//import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
//import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
//@TeleOp
//public class Test extends LinearOpMode {
//    MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
//
//    @Override
//    public void runOpMode(){
//            waitForStart();
//
//            while (opModeIsActive()) {
//                drive.fieldDrive(
//                new Pose2d(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x)
//);
//                drive.updatePoseEstimate();
//
//            }
//    }
//
//}
