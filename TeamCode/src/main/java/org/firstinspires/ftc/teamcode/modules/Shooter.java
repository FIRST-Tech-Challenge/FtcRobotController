package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Shooter {

    public final DcMotorEx shooterUp;
    public final DcMotorEx shooterDown;

    final VoltageSensor batteryVoltage;

    public static double target = 0;

    final double TICKS_PER_REV = 28;
    double MAX_RPM = 0;
    double GEAR_RATIO = 1;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0,0,0,0);

    public Shooter(LinearOpMode opMode) {
        shooterUp = opMode.hardwareMap.get(DcMotorEx.class, "shooterUp");
        shooterDown = opMode.hardwareMap.get(DcMotorEx.class, "shooterDown");

        batteryVoltage = opMode.hardwareMap.voltageSensor.iterator().next();

        shooterDown.setDirection(DcMotorEx.Direction.REVERSE);

        shooterUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setPIDFCoefficients(shooterUp, pidfCoefficients);
        setPIDFCoefficients(shooterDown, pidfCoefficients);
    }

    public void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltage.getVoltage())
        );
    }

    public void setPower(double power) {
        shooterUp.setPower(power);
        shooterDown.setPower(power);
    }

    public void setTarget(double newTarget) {
        target = newTarget;
    }

    public void setVelocity() {
        shooterUp.setVelocity(target);
        shooterDown.setVelocity(target);
    }

    public double getVelocity() {
        return  shooterUp.getVelocity();
    }
    public double convertTicksToRPS(double ticks_per_rev){
        return ticks_per_rev / TICKS_PER_REV;
    }

}
