package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.rustlib.commandsystem.InstantCommand;
import org.rustlib.commandsystem.Subsystem;
import org.rustlib.commandsystem.Trigger;
import org.rustlib.control.PIDController;
import org.rustlib.hardware.PairedEncoder;
import org.rustlib.rustboard.RustboardServer;

public class Slide extends Subsystem {

    public final DcMotor motor1;
    public final DcMotor motor2;
    public final PairedEncoder encoder;
    private final PIDController controller;
    private final TouchSensor limit;
    private final Placer placer;
    private double feedforward = 0;
    private int targetPosition = 0;
    private double lastSpeed = 0;
    private double lastInput = 0;

    public Slide(HardwareMap hardwareMap, Placer placer) {
        motor1 = hardwareMap.get(DcMotor.class, "slideMotor1");
        motor2 = hardwareMap.get(DcMotor.class, "slideMotor2");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = new PairedEncoder(hardwareMap.get(DcMotor.class, "lf"), false);
        encoder.reset();
        limit = hardwareMap.get(TouchSensor.class, "limit");
        this.placer = placer;
        controller = new PIDController(0.0017, 0.0000008, 0.000003);
        controller.resetIntegralOnSetPointChange = true;

        new Trigger(() -> encoder.getTicks() > SubsystemConstants.Slide.preparePlacerPosition && encoder.ticksPerSecond() > 0).onTrue(new InstantCommand(placer::placePosition));
        new Trigger(() -> encoder.getTicks() < SubsystemConstants.Slide.stowPlacerPosition && targetPosition < 10 || encoder.ticksPerSecond() < -400).onTrue(new InstantCommand(placer::storagePosition));
    }

    private static boolean gamepadActive(double input) {
        return Math.abs(input) > 0.05;
    }

    public void mizoom(double input) {
        double calculatedSpeed;
        if (gamepadActive(input) && !(input > 0 && encoder.getTicks() > SubsystemConstants.Slide.maxExtensionPosition)) { // If manual control is both requested and allowed
            calculatedSpeed = input + feedforward;
            lastInput = input;
        } else { // If automatic control is requested or manual control is not allowed
            if (gamepadActive(lastInput)) {
                targetPosition = encoder.getTicks();
            }
            calculatedSpeed = controller.calculate(encoder.getTicks(), targetPosition) + feedforward;
            if (encoder.getTicks() < 125 && targetPosition < 80) {
                calculatedSpeed -= 0.5;
            }
            lastInput = 0;
        }
        calculatedSpeed = applyAccelerationLimits(calculatedSpeed);
        RustboardServer.setNodeValue("slide speed", calculatedSpeed);
        drive(calculatedSpeed);
    }

    private double applyAccelerationLimits(double speed) {
        double accelMax = RustboardServer.getDoubleValue("slide accel", 0.5);
        ;
        if (speed > 0) {
            speed = Math.min(lastSpeed + accelMax, speed);
        } else {
            speed = Math.max(lastSpeed - accelMax, speed);
        }
        return speed;
    }

    private void drive(double speed) {
        motor1.setPower(speed);
        motor2.setPower(-speed);
        lastSpeed = speed;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = Math.min(targetPosition, SubsystemConstants.Slide.maxExtensionPosition);
    }

    public boolean atTargetPosition() {
        return Math.abs(targetPosition - encoder.getTicks()) < SubsystemConstants.Slide.maxTargetError;
    }

    @Override
    public void periodic() {
        if (limit.isPressed()) {
            encoder.reset();
            placer.close();
            targetPosition = Math.max(targetPosition, 0);
        }

        controller.setP(RustboardServer.getDoubleValue("slide kP", 0.0014));
        controller.setI(RustboardServer.getDoubleValue("slide kI", 0.0));
        controller.setD(RustboardServer.getDoubleValue("slide kD", 0.0008));
        feedforward = RustboardServer.getDoubleValue("slide feedforward", 0.2);
        ;
        RustboardServer.log(controller.getGains().toString());
        RustboardServer.log("feedforward: " + feedforward);
        RustboardServer.setNodeValue("slide pose", encoder.getTicks());
        RustboardServer.setNodeValue("slide velocity", encoder.ticksPerSecond());
        RustboardServer.setNodeValue("slide target", targetPosition);
        RustboardServer.setNodeValue("slide limit", limit.isPressed());
    }
}
