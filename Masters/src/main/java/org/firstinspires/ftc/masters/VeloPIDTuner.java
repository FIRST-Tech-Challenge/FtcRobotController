package org.firstinspires.ftc.masters;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.masters.oldAndUselessStuff.TuningController;

@Config
@TeleOp
public class VeloPIDTuner extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 0);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");

        // Reverse as appropriate
        // myMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

        TuningController tuningController = new TuningController();

        double lastKp = 0.0;
        double lastKi = 0.0;
        double lastKd = 0.0;
        double lastKf = getMotorVelocityF();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        tuningController.start();

        while (!isStopRequested() && opModeIsActive()) {
            double targetVelo = tuningController.update();
            myMotor.setVelocity(targetVelo);

            telemetry.addData("targetVelocity", targetVelo);

            double motorVelo = myMotor.getVelocity();
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelo - motorVelo);

            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            tuningController.update();
            telemetry.update();
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV);
    }
}