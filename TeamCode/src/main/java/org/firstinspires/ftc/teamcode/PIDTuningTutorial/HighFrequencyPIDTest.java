package org.firstinspires.ftc.teamcode.PIDTuningTutorial;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

@TeleOp
@Config
@Disabled
public class HighFrequencyPIDTest extends LinearOpMode {
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    public static double TESTING_MAX_SPEED = 0.9 * MOTOR_MAX_RPM;
    public static double TESTING_MIN_SPEED = 0.3 * MOTOR_MAX_RPM;

    // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
    // alphabetical order. Thus, we preserve the actual order of the process
    public static double STATE1_RAMPING_UP_DURATION = 3.5;
    public static double STATE2_COASTING_1_DURATION = 4;
    public static double STATE3_RAMPING_DOWN_DURATION = 2;
    public static double STATE4_COASTING_2_DURATION = 2;
    public static double STATE5_RANDOM_1_DURATION = 2;
    public static double STATE6_RANDOM_2_DURATION = 2;
    public static double STATE7_RANDOM_3_DURATION = 2;
    public static double STATE8_REST_DURATION = 1;

    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0061, 0, 0.000035);
    public static double kV = 0.000365;
//    public static double kV = 1 / rpmToTicksPerSecond(5400);
    public static double kA = 0.00015;
    public static double kStatic = 0;

    enum State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2,
        RANDOM_1,
        RANDOM_2,
        RANDOM_3,
        REST
    }

    private double currentTargetVelo = 0.0;
    private double lastTargetVelo = 0.0;

    private ElapsedTime veloTimer = new ElapsedTime();
    private boolean firstVeloRead = true;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        myMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

        double lastKv = kV;
        double lastKa = kA;
        double lastKstatic = kStatic;

        double SPEED_RANGE = TESTING_MAX_SPEED - TESTING_MIN_SPEED;

        ElapsedTime externalTimer = new ElapsedTime();

        ElapsedTime loopTimer = new ElapsedTime();
        MovingStatistics loopStats = new MovingStatistics(20);

        StateMachine stateMachine = new StateMachineBuilder<State>()
                .state(State.RAMPING_UP)
                .transitionTimed(STATE1_RAMPING_UP_DURATION)
                .onEnter(externalTimer::reset)
                .loop(() -> {
                    double progress = externalTimer.seconds() / STATE1_RAMPING_UP_DURATION;
                    double target = progress * SPEED_RANGE + TESTING_MIN_SPEED;

                    currentTargetVelo = rpmToTicksPerSecond(target);
                })

                .state(State.COASTING_1)
                .transitionTimed(STATE2_COASTING_1_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(TESTING_MAX_SPEED))

                .state(State.RAMPING_DOWN)
                .transitionTimed(STATE3_RAMPING_DOWN_DURATION)
                .onEnter(externalTimer::reset)
                .loop(() -> {
                    double progress = externalTimer.seconds() / STATE3_RAMPING_DOWN_DURATION;
                    double target = TESTING_MAX_SPEED - progress * SPEED_RANGE;

                    currentTargetVelo = rpmToTicksPerSecond(target);
                })

                .state(State.COASTING_2)
                .transitionTimed(STATE4_COASTING_2_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(TESTING_MIN_SPEED))

                .state(State.RANDOM_1)
                .transitionTimed(STATE5_RANDOM_1_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(Math.random() * SPEED_RANGE + TESTING_MIN_SPEED))

                .state(State.RANDOM_2)
                .transitionTimed(STATE6_RANDOM_2_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(Math.random() * SPEED_RANGE + TESTING_MIN_SPEED))

                .state(State.RANDOM_3)
                .transitionTimed(STATE7_RANDOM_3_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(Math.random() * SPEED_RANGE + TESTING_MIN_SPEED))

                .state(State.REST)
                .transitionTimed(STATE8_REST_DURATION)
                .onEnter(() -> currentTargetVelo = 0)

                .exit(State.RAMPING_UP)

                .build();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        stateMachine.setLooping(true);
        stateMachine.start();

        loopTimer.reset();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("currentState", stateMachine.getState());

            veloController.setTargetVelocity(currentTargetVelo);

            if(firstVeloRead) {
                firstVeloRead = false;

                lastTargetVelo = currentTargetVelo;
                veloTimer.reset();
            } else {
                veloController.setTargetAcceleration((currentTargetVelo - lastTargetVelo) / veloTimer.seconds());

                lastTargetVelo = currentTargetVelo;
                veloTimer.reset();
            }

            telemetry.addData("targetVelocity", currentTargetVelo);

            double motorVelo = myMotor.getVelocity();
            Log.i("velo", Double.toString(motorVelo));
            Log.i("pos", Double.toString(myMotor.getCurrentPosition()));

            myMotor.setPower(veloController.update(myMotor.getCurrentPosition(), motorVelo));

            if(lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                lastKv = kV;
                lastKa = kA;
                lastKstatic = kStatic;

                veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
            }

            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", currentTargetVelo - motorVelo);

            telemetry.addData("upperBound", rpmToTicksPerSecond(TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);

            Log.i("loopTime", Double.toString(loopTimer.milliseconds()));
            loopStats.add(loopTimer.milliseconds());
            loopTimer.reset();

            Log.i("loopAvg", Double.toString(1000 / loopStats.getMean()));

            stateMachine.update();
            telemetry.update();
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
