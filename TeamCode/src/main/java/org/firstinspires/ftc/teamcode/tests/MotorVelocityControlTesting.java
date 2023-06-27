package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.robotbase.MotorExEx;

import java.util.List;

@Config
@TeleOp(name="Motor Velocity Controlled Test", group="Tests")
@Disabled
public class MotorVelocityControlTesting extends LinearOpMode {
    MotorExEx frontLeft;
    MotorExEx frontRight;
    MotorExEx rearLeft;
    MotorExEx rearRight;
//    MotorGroup group;
    MecanumDrive mecanum;
    GamepadEx driverOp;

    Telemetry dahsboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    public static double KP = 1.1;
    public static double KI = 2.7;
    public static double KD = 0;
    public static double A_KD = 0.5;

    public static double A = 0.3;

    double last_KP=KP, last_KI=KI, last_KD=KD;

    public static double minIntegralBound = -400;
    public static double maxIntegralBound = 400;

    private double vel_1 = 0, vel_2 = 0, vel_3 = 0, vel_4 = 0;

    List<Double> dist;

    @Override
    public void runOpMode() {
        frontLeft = new MotorExEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorExEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        rearLeft = new MotorExEx(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
        rearRight = new MotorExEx(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        rearLeft.setRunMode(Motor.RunMode.VelocityControl);
        rearRight.setRunMode(Motor.RunMode.VelocityControl);

//        minIntegralBound = -frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;
//        maxIntegralBound = frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;

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

        driverOp = new GamepadEx(gamepad1);

        frontLeft.setInverted(true);
        rearLeft.setInverted(true);

//        mecanum = new MecanumDrive(frontLeft, frontRight,
//                rearLeft, rearRight);

        waitForStart();

        while(opModeIsActive()) {
//            mecanum.driveRobotCentric(
//                    -driverOp.getLeftX(),
//                    -driverOp.getLeftY(),
//                    -driverOp.getRightX()
//            );

            vel_1 = A*frontLeft.getCorrectedVelocity() + (1-A)*vel_1;
            vel_2 = A*frontRight.getCorrectedVelocity() + (1-A)*vel_2;
            vel_3 = A*rearLeft.getCorrectedVelocity() + (1-A)*vel_3;
            vel_4 = A*rearRight.getCorrectedVelocity() + (1-A)*vel_4;

            if(KP != last_KP || KI != last_KI || KD != last_KD) {
                frontLeft.setVeloCoefficients(KP, KI, KD);
                frontRight.setVeloCoefficients(KP, KI, KD);
                rearLeft.setVeloCoefficients(KP, KI, KD);
                rearRight.setVeloCoefficients(KP, KI, KD);

                last_KP = KP;
                last_KI = KI;
                last_KD = KD;
            }

//            frontLeft.setVeloCoefficients(KP, KI, KD);
//            frontRight.setVeloCoefficients(KP, KI, KD);
//            rearLeft.setVeloCoefficients(KP, KI, KD);
//            rearRight.setVeloCoefficients(KP, KI, KD);
            frontLeft.set(driverOp.getLeftY());
            frontRight.set(driverOp.getLeftY());
            rearLeft.set(driverOp.getLeftY());
            rearRight.set(driverOp.getLeftY());

            frontLeft.setBuffer(1);
            frontRight.setBuffer(1);
            rearLeft.setBuffer(1);
            rearRight.setBuffer(1);

            frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
            frontRight.setIntegralBounds(minIntegralBound, maxIntegralBound);
            rearLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
            rearRight.setIntegralBounds(minIntegralBound, maxIntegralBound);

            dahsboardTelemetry.addData("Actual Velo 1: ", vel_1);
            dahsboardTelemetry.addData("Actual Velo 2: ", vel_2);
            dahsboardTelemetry.addData("Actual Velo 3: ", vel_3);
            dahsboardTelemetry.addData("Actual Velo 4: ", vel_4);

            dahsboardTelemetry.addData("P: ", KP);
            dahsboardTelemetry.addData("I: ", KI);
            dahsboardTelemetry.addData("D: ", KD);

            dahsboardTelemetry.addData("Top Target: ", frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND*-driverOp.getLeftY());
            dahsboardTelemetry.addData("low Target: ", frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND*driverOp.getLeftY());
            dahsboardTelemetry.update();
        }
    }
}

