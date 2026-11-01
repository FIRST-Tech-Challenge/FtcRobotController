package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * INTAKE - picks up game pieces.
 *
 * toggle() flips the motor between off and RobotConstants.INTAKE_POWER (tested on the robot).
 * TODO: reverse() to unjam, and a sensor to know when a piece is held.
 */
public class Intake implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private DcMotor motor;
    private boolean running = false;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        telemetry.addData("Intake", running ? "on" : "off");
    }

    @Override
    public void stop() {
        off();
    }

    /** Flips the intake between off and RobotConstants.INTAKE_POWER. */
    public void toggle() {
        running = !running;
        motor.setPower(running ? RobotConstants.INTAKE_POWER : 0);
    }

    public void off() {
        running = false;
        motor.setPower(0);
    }
}
