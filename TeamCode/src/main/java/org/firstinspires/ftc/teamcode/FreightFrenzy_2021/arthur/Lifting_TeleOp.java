package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.arthur;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Lifting", group="Linear OpMode")
public class Lifting_TeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final int TICKS_PER_ROTATION = 537;
    private final double LENGTH_LIFTER = 0.95; // in meters
    private final double WHEEL_RADIUS = 0.019; // in meters
    private final double WHEEL_DIAMETER = WHEEL_RADIUS * 2 * Math.PI;

    private DcMotor lifterMotor;

    private GamepadState gamepad;

    public static final double SPEED_RATE_INTERVAL = 0.1;

    public Action liftUp = () -> {
        int ticks = lifterMotor.getCurrentPosition();
        telemetry.addData("ticks", ticks);
        double revs = (double)ticks / TICKS_PER_ROTATION;
        double distance = revs * WHEEL_DIAMETER;
        telemetry.addData("distance", distance);
        if (distance < LENGTH_LIFTER) {
            lifterMotor.setPower(0.5);
        } else {
            lifterMotor.setPower(0);
        }
    };

    public Action liftDown = () -> {
        int ticks = lifterMotor.getCurrentPosition();
        telemetry.addData("ticks", ticks);
        int revs = ticks / TICKS_PER_ROTATION;
        double distance = revs * WHEEL_DIAMETER;
        telemetry.addData("distance", distance);
        if (distance > 0) {
            lifterMotor.setPower(-0.5);
        } else {
            lifterMotor.setPower(0);
        }
    };

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            execute();
        }

        finish();
    }

    public void execute() {
        runtime.reset();

        gamepad.onButtonOnce("y", liftUp);

        gamepad.onButtonOnce("a", liftDown);

        printData();
        telemetry.update();
    }

    private void printData() {
        telemetry.addData("lifter power", lifterMotor.getPower());
    }

    private static final double DEFAULT_SPEED_RATE = 0.5;

    private void initialize() {
        gamepad = new GamepadState(gamepad1);
        lifterMotor = hardwareMap.get(DcMotor.class, "Slide");
        MotorConfigurationType lifterMotorConf = new MotorConfigurationType();
        lifterMotorConf.setTicksPerRev(TICKS_PER_ROTATION);
        lifterMotor.setMotorType(new MotorConfigurationType());
        lifterMotor.setDirection(DcMotor.Direction.FORWARD);
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    private void finish() {
        telemetry.addData("status", "finished");
        telemetry.addData("runtime", runtime.toString());
        telemetry.update();
    }

}
