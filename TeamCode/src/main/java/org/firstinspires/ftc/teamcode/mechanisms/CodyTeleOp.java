 package org.firstinspires.ftc.teamcode.mechanisms;

 import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
 import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
 import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.HardwareMap;
 import com.qualcomm.robotcore.hardware.IMU;

 import fi.iki.elonen.NanoHTTPD;

 @TeleOp(name = "CodyDrive")
 public class CodyTeleOp extends LinearOpMode {
     @Override
     public void runOpMode() {

         DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
         DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
         DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
         DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "BR");
         CRServo RightContinuousServo = hardwareMap.get(CRServo.class, "RightNTKServo");
         CRServo LeftContinuousServo = hardwareMap.get(CRServo.class, "LeftNTKServo");
         DcMotor IntakeMotor = hardwareMap.get(DcMotor.class, "NTK");
         DcMotor ShooterMotor = hardwareMap.get(DcMotor.class, "Shooter");


         backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

     waitForStart();
     if (isStopRequested()) return;
     while (opModeIsActive()){
         double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
         double rx = gamepad1.right_stick_x;

         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.6); // 0.45 for non-gripy wheels, 0.6 for gripy wheels
         double frontLeftPower = (y + x + rx) / denominator;
         double backLeftPower = (y - x + rx) / denominator;
         double frontRightPower = (y - x - rx) / denominator;
         double backRightPower = (y + x - rx) / denominator;

         frontLeftMotor.setPower(frontLeftPower);
         backLeftMotor.setPower(backLeftPower);
         frontRightMotor.setPower(frontRightPower);
         backRightMotor.setPower(backRightPower);}}}

