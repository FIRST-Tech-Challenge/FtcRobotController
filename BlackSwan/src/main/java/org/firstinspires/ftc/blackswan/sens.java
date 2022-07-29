//package org.firstinspires.ftc.blackswan;
//
//import android.os.Build;
//
//import androidx.annotation.RequiresApi;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//@TeleOp(name = "Test color sensors", group ="test")
//public class sens extends LinearOpMode {
//
//    Robot robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot = new Robot(hardwareMap, telemetry, this);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
////            telemetry.addData("Left Light Level: ", robot.colorSensorLeft.alpha());
////            telemetry.addData("Left Red: ", robot.colorSensorLeft.red());
////            telemetry.addData("Left Blue: ", robot.colorSensorLeft.blue());
////            telemetry.addData("Left Green: ", robot.colorSensorLeft.green());
////
////            telemetry.addData("Right Light Level: ", robot.colorSensorRight.alpha());
////            telemetry.addData("Right Red: ", robot.colorSensorRight.red());
////            telemetry.addData("Right Blue: ", robot.colorSensorRight.blue());
////            telemetry.addData("Right Green: ", robot.colorSensorRight.green());
//
//            telemetry.addData("Back Light Level: ", robot.colorSensorBack.alpha());
//            telemetry.addData("Back Red: ", robot.colorSensorBack.red());
//            telemetry.addData("Back Blue: ", robot.colorSensorBack.blue());
//            telemetry.addData("Back Green: ", robot.colorSensorBack.green());
//
//            telemetry.update();
//        }
//    }
//}
