package org.firstinspires.ftc.teamcode.PIDTuningTutorial;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
@Disabled

@Config
@TeleOp
public class ShootingConsistencyTest extends LinearOpMode {
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    public static boolean RUN_USING_ENCODER = false;
    public static boolean DEFAULT_GAINS = false;

    public static double TESTING_SPEED = 0.9 * MOTOR_MAX_RPM;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30, 0, 2, 15);

    private double lastKp = 0.0;
    private double lastKi = 0.0;
    private double lastKd = 0.0;
    private double lastKf = getMotorVelocityF();

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        myMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        if (RUN_USING_ENCODER)
            myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            setVelocity(myMotor, TESTING_SPEED);

            printVelocity(myMotor, TESTING_SPEED);

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }

    private void printVelocity(DcMotorEx motor, double target) {
        telemetry.addData("targetVelocity", rpmToTicksPerSecond(target));

        double motorVelo = motor.getVelocity();
        telemetry.addData("velocity", motorVelo);
        telemetry.addData("error", rpmToTicksPerSecond(target) - motorVelo);

        telemetry.addData("upperBound", rpmToTicksPerSecond(TESTING_SPEED) * 1.15);
        telemetry.addData("lowerBound", 0);
    }

    private void setVelocity(DcMotorEx motor, double power) {
        if(RUN_USING_ENCODER) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        }
        else {
            Log.i("mode", "setting power");
            motor.setPower(power / MOTOR_MAX_RPM);
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV);
    }
}
