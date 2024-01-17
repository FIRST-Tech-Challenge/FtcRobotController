package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Zayne;//package org.firstinspires.ftc.teamcode.TeleOps.Memdev;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//@TeleOp
//public class ZayneWinch extends LinearOpMode {
//    private DcMotor Winchie;
//    private Servo servie;
//    private int zero = 0;
//    private double X = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor winch = hardwareMap.dcMotor.get("Winch");
//        Servo lock = hardwareMap.servo.get("Lock");
//        Gamepad g1 = new Gamepad();
//
//        waitForStart();
//        while (opModeIsActive()) {
//            if (g1.x == true) {
//                lock.setPosition(45);
//                winch.setPower(g1.x);
//
//            } else if (g1.x == false){
//                lock.setPosition(0);
//                wait(.5);
//                winch.setPower(0);
//            }
//
//        }
//    }
//
//
//}
