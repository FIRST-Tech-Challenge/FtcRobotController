package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * FoundationHook spec:
 */
public class FoundationHook extends Logger<FoundationHook> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo rightHook;
    private AdjustableServo leftHook;
    /*private*/ public TouchSensor magTouch;
    public DistanceSensor rangetouch;


    private final double LEFT_HOOK_INIT = 0.08;
    private final double LEFT_HOOK_UP = 0.141;
    private final double LEFT_HOOK_DOWN = 0.615;

    private final double RIGHT_HOOK_INIT = 0.8;
    private final double RIGHT_HOOK_UP = 0.739;
    private final double RIGHT_HOOK_DOWN = .273;

    private boolean hookIsDown = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "foundationHook";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public FoundationHook(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (leftHook != null)
            hookInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        leftHook = new AdjustableServo(0, 1).configureLogging(
                logTag + ":foundationHook", logLevel
        );
        leftHook.configure(configuration.getHardwareMap(), "leftHook");
        configuration.register(leftHook);

        rightHook = new AdjustableServo(0, 1).configureLogging(
                logTag + ":foundationHook", logLevel
        );
        rightHook.configure(configuration.getHardwareMap(), "rightHook");
        configuration.register(rightHook);

        magTouch = configuration.getHardwareMap().touchSensor.get("mag_touch");
        rangetouch = configuration.getHardwareMap().get(DistanceSensor.class, "backRange");
        hookInit();
        // configuration.register(this);
    }

    public void hookInit() {
        leftHook.setPosition(LEFT_HOOK_INIT);
        rightHook.setPosition(RIGHT_HOOK_INIT);
        hookIsDown = false;
        //hookUp();
        // configuration.register(this);
    }

    public boolean touchingState() {
        return magTouch.isPressed();
    }

    public double rangeReading() {
        return rangetouch.getDistance(DistanceUnit.CM);
    }

    public void hookUp() {
        leftHook.setPosition(LEFT_HOOK_UP);
        rightHook.setPosition(RIGHT_HOOK_UP);
        hookIsDown = false;
    }

    public void hookDown() {
        leftHook.setPosition(LEFT_HOOK_DOWN);
        rightHook.setPosition(RIGHT_HOOK_DOWN);
        hookIsDown = true;
    }

    public void hookAuto() {
        if (hookIsDown) {
            hookUp();
        } else {
            hookDown();
        }
    }


    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (leftHook != null) {
            line.addData("Left Hook", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return leftHook.getPosition();
                }
            });
        }

        if (rightHook != null) {
            line.addData("Right Hook", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return rightHook.getPosition();
                }
            });
        }
    }

}



