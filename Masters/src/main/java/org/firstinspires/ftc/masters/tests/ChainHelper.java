//package org.firstinspires.ftc.masters.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp(name = "ChainHelper V3, Boogaloo Edition")
//public class ChainHelper extends LinearOpMode {
//
//    public void runOpMode() throws InterruptedException {
//
//        DcMotor hang = hardwareMap.dcMotor.get("hangingMotor");
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//
//            if(gamepad1.dpad_up){
//                hang.setPower(.2);
//            } else if (gamepad1.dpad_down) {
//                hang.setPower(-.2);
//            } else {
//                hang.setPower(0);
//            }
//
//        }
//    }
//
//}
