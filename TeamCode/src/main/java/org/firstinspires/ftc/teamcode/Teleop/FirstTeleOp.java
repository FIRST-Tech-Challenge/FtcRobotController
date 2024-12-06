package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.IguanasaurusFunctions;

@TeleOp(name = "FirstTeleop")
public class FirstTeleOp extends LinearOpMode {
    DcMotorEx lb, lf, rb, rf, intake, vertical1, vertical2;
    Servo inClaw, depClaw, inWrist, depWrist, transfer;
    ColorSensor transColor, inColor;
    IMU imu;

    public static double IN_CLAW_CLOSE = 0.0;
    public static double IN_CLAW_OPEN = 1.0;


    boolean pressA = false;


    boolean InClawOpen = true;
    boolean hasSample = false;

    @Override
    public void runOpMode() throws InterruptedException {
        lb         = hardwareMap.get(DcMotorEx .class, "lb");
        lf         = hardwareMap.get(DcMotorEx.class, "lf");
        rb         = hardwareMap.get(DcMotorEx.class, "rb");
        rf         = hardwareMap.get(DcMotorEx.class, "rf");
        intake     = hardwareMap.get(DcMotorEx.class, "intake");
        vertical1  = hardwareMap.get(DcMotorEx.class, "vertical1");
        vertical2  = hardwareMap.get(DcMotorEx.class, "vertical2");
        inClaw     = hardwareMap.get(Servo.class, "inClaw");
        depClaw    = hardwareMap.get(Servo.class, "depClaw");
        inWrist    = hardwareMap.get(Servo.class, "inWrist");
        depWrist   = hardwareMap.get(Servo.class, "depWrist");
        transfer   = hardwareMap.get(Servo.class, "transfer");
        imu        = hardwareMap.get(IMU.class, "imu");

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake Claw Toggle
            if (gamepad1.a && !pressA) {
                    pressA = true;
                }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }
        }
    }

    public void
}
