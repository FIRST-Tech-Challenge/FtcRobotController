package org.firstinspires.ftc.teamcode.hardware.MechBot;

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
public class BottomWobbleGoalGrabber extends Logger<BottomWobbleGoalGrabber> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo pivot;
    private AdjustableServo grabber;
    /*private*/ public TouchSensor magTouch;
    public DistanceSensor rangetouch;


    private final double PIVOT_UP = 0.9;
    private final double PIVOT_INIT = PIVOT_UP;
    private final double PIVOT_DOWN = 0.25;

    private final double GRABBER_OPEN = 0.6;
    private final double GRABBER_INIT = GRABBER_OPEN;
    private final double GRABBER_CLOSE = 0.4;

    private boolean pivotIsDown = false;
    private boolean grabberIsClosed = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "bottomWobbleGoalGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public BottomWobbleGoalGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (grabber != null)
            servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        grabber = new AdjustableServo(0, 1).configureLogging(
                logTag + ":bottomWobbleGoalGrabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);

        pivot = new AdjustableServo(0, 1).configureLogging(
                logTag + ":bottomWobbleGoalGrabber", logLevel
        );
        pivot.configure(configuration.getHardwareMap(), "pivot");
        configuration.register(pivot);

        magTouch = configuration.getHardwareMap().touchSensor.get("mag_touch");
        rangetouch = configuration.getHardwareMap().get(DistanceSensor.class, "backRange");
        servoInit();
        // configuration.register(this);
    }

    public void servoInit() {
        grabber.setPosition(PIVOT_INIT);
        pivot.setPosition(GRABBER_INIT);
        pivotIsDown = false;
        grabberIsClosed = false;
        //hookUp();
        // configuration.register(this);
    }

    public boolean touchingState() {
        return magTouch.isPressed();
    }

    public double rangeReading() {
        return rangetouch.getDistance(DistanceUnit.CM);
    }

    public void pivotUp() {
        pivot.setPosition(PIVOT_UP);
        pivotIsDown = false;
    }

    public void pivotDown() {
        pivot.setPosition(PIVOT_DOWN);
        pivotIsDown = true;
    }

    public void pivotAuto() {
        if (pivotIsDown) {
            pivotUp();
        } else {
            pivotDown();
        }
    }

    public void grabberOpen(){
        grabber.setPosition(GRABBER_OPEN);
        grabberIsClosed = false;
    }

    public void grabberClose(){
        grabber.setPosition(GRABBER_CLOSE);
        grabberIsClosed = true;
    }

    public void grabberAuto(){
        if (grabberIsClosed) {
            grabberOpen();
        } else {
            grabberClose();
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

        if (grabber != null) {
            line.addData("Grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }

        if (pivot != null) {
            line.addData("Pivot", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return pivot.getPosition();
                }
            });
        }
    }

}



