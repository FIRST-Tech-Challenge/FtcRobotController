package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.inventors.ftc.robotbase.MotorExEx;

@TeleOp
@Disabled
public class MecanumTest extends LinearOpMode {
    private MotorExEx frontLeft, frontRight, rearRight, rearLeft;
    private MecanumDrive drive;

    public static double KP = 1.1;
    public static double KI = 2.7;
//    public static double KP = 0;
//    public static double KI = 0;
    public static double KD = 0;
    public static double minIntegralBound = -400;
    public static double maxIntegralBound = 400;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new MotorExEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorExEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        rearRight = new MotorExEx(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        rearLeft = new MotorExEx(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);

        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        rearRight.setRunMode(Motor.RunMode.VelocityControl);
        rearLeft.setRunMode(Motor.RunMode.VelocityControl);

        frontLeft.setVeloCoefficients(KP, KI, KD);
        frontLeft.setFeedforwardCoefficients(150, 1.1, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
        frontRight.setVeloCoefficients(KP, KI, KD);
        frontRight.setFeedforwardCoefficients(120, 0.97, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
        rearLeft.setVeloCoefficients(KP, KI, KD);
        rearLeft.setFeedforwardCoefficients(120, 1, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
        rearRight.setVeloCoefficients(KP, KI, KD);
        rearRight.setFeedforwardCoefficients(220, 1.07, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);

//        frontLeft.setInverted(true);
//        frontRight.setInverted(true);
//        rearRight.setInverted(true);
//        rearLeft.setInverted(true);

        drive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);
//        drive.setMaxSpeed();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Hello", "");
            telemetry.update();
            drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
