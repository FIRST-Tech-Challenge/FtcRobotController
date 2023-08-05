package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Disabled
@Config
@TeleOp (name = "SliderConstantsTesting", group = "Tests")
public class SliderConstantsTesting extends LinearOpMode {
    MotorEx sliderMotor;
    GamepadEx driverOp;

    Telemetry dahsboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    public static double KP = 1.1;
    public static double KS = 0, KV = 0, KA = 0;

    public static double A = 0.3;

    double last_KP=KP, last_KS=KS, last_KV=KV, last_KA=KA;

    private double vel = 0;

    @Override
    public void runOpMode() {
        sliderMotor = new MotorEx(hardwareMap, "slider", Motor.GoBILDA.RPM_312);
        sliderMotor.setRunMode(Motor.RunMode.VelocityControl);

//        sliderMotor.setPositionCoefficient(KP);
        sliderMotor.setFeedforwardCoefficients(0, 0, 0);//2795
        driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while(opModeIsActive()) {
            vel = (1-A)*sliderMotor.getCorrectedVelocity() + A*vel;

            if(KP != last_KP || KS != last_KS || KV != last_KV || KA != last_KA) {
//                sliderMotor.setPositionCoefficient(KP);
                sliderMotor.setFeedforwardCoefficients(KS, KV, KA);

                last_KP = KP;
                last_KS = KS;
                last_KV = KV;
                last_KA = KA;
            }

            sliderMotor.set(driverOp.getLeftY()/2);

            sliderMotor.setBuffer(1);

            dahsboardTelemetry.addData("Actual Velocity: ", vel);

            dahsboardTelemetry.addData("P: ", KP);
            dahsboardTelemetry.addData("KS: ", KS);
            dahsboardTelemetry.addData("KV: ", KV);
            dahsboardTelemetry.addData("KA: ", KA);

            dahsboardTelemetry.addData("Top Target: ", sliderMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND*-driverOp.getLeftY());
            dahsboardTelemetry.update();
        }
    }
}
