package org.firstinspires.ftc.teamcode.PIDTuningTutorial;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled

@Config
@TeleOp
public class RawMotorDemo extends LinearOpMode {
    public static double MOTOR_TICKS_PER_REV = 383.6;
    public static double MOTOR_MAX_RPM = 435;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    public static double STATE1_RAMPING_UP_DURATION = 3.5;
    public static double STATE2_COASTING_1_DURATION = 4;
    public static double STATE3_RAMPING_DOWN_DURATION = 2;
    public static double STATE4_COASTING_2_DURATION = 2;

    public static double TESTING_MAX_SPEED = 0.5;
    public static double TESTING_MIN_SPEED = 0.3;

    enum State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2
    }

    private State currentState;
    private State lastState;
    private ElapsedTime timer;

    private double currentTargetVelo = 0.0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.RAMPING_UP;
        lastState = null;
        timer = new ElapsedTime();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("currentState", currentState);

            switch (currentState) {
                case RAMPING_UP:
                    double progress1 = timer.seconds() / STATE1_RAMPING_UP_DURATION;
                    double target1 = progress1 * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;

                    currentTargetVelo = target1;
                    myMotor.setPower(currentTargetVelo);

                    if (progress1 >= 1) {
                        currentState = State.COASTING_1;
                        timer.reset();
                    }
                    break;
                case COASTING_1:
                    if (lastState != State.COASTING_1) {
                        currentTargetVelo = TESTING_MAX_SPEED;
                        myMotor.setPower(currentTargetVelo);

                        lastState = State.COASTING_1;
                    }

                    if (timer.seconds() >= STATE2_COASTING_1_DURATION) {
                        currentState = State.RAMPING_DOWN;
                        timer.reset();
                    }
                    break;
                case RAMPING_DOWN:
                    double progress2 = timer.seconds() / STATE3_RAMPING_DOWN_DURATION;
                    double target2 = TESTING_MAX_SPEED - progress2 * (TESTING_MAX_SPEED - TESTING_MIN_SPEED);

                    currentTargetVelo = target2;
                    myMotor.setPower(currentTargetVelo);

                    if (progress2 >= 1) {
                        currentState = State.COASTING_2;
                        timer.reset();
                    }
                    break;
                case COASTING_2:
                    if (lastState != State.COASTING_2) {
                        currentTargetVelo = TESTING_MIN_SPEED;
                        myMotor.setPower(TESTING_MIN_SPEED);

                        lastState = State.COASTING_2;
                    }

                    if (timer.seconds() >= STATE4_COASTING_2_DURATION) {
                        currentState = State.RAMPING_UP;
                        timer.reset();
                    }
                    break;
            }

            double targetTicksPerSec = rpmToTicksPerSecond(currentTargetVelo * MOTOR_MAX_RPM);
            telemetry.addData("targetVelocity", targetTicksPerSec);

            double motorVelo = myMotor.getVelocity();
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetTicksPerSec - motorVelo);

            telemetry.addData("upperBound", rpmToTicksPerSecond(MOTOR_MAX_RPM * TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);

            telemetry.update();
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }
}
