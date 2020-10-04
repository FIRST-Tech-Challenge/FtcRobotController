package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.CRServo;
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
public class Hopper extends Logger<Hopper> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo feeder;
    private CRServo transfer;
    /*private*/ public TouchSensor magTouch;
    public DistanceSensor rangetouch;


    private final double FEEDER_IN = 0.9;
    private final double FEEDER_INIT = FEEDER_IN;
    private final double FEEDER_OUT = 0.25;

    private boolean feederIsIn = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "hopper";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public Hopper(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (transfer != null)
            servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        transfer = configuration.getHardwareMap().get(CRServo.class, "hopper");

        feeder = new AdjustableServo(0, 1).configureLogging(
                logTag + ":bottomWobbleGoalGrabber", logLevel
        );
        feeder.configure(configuration.getHardwareMap(), "feeder");
        configuration.register(feeder);
        servoInit();
      // configuration.register(this);
    }

    public void servoInit() {
        transfer.setPower(0);
        feeder.setPosition(FEEDER_INIT);
        feederIsIn = false;
        //hookUp();
        // configuration.register(this);
    }

    public boolean touchingState() {
        return magTouch.isPressed();
    }

    public double rangeReading() {
        return rangetouch.getDistance(DistanceUnit.CM);
    }

    public void feederIn() {
        feeder.setPosition(FEEDER_IN);
        feederIsIn = true;
    }

    public void feederOut() {
        feeder.setPosition(FEEDER_OUT);
        feederIsIn = false;
    }

    public void feederAuto() {
        if (feederIsIn) {
            feederIn();
        } else {
            feederOut();
        }
    }

    public void transferUp(){
        transfer.setPower(1);
    }

    public void transferDown(){
        transfer.setPower(-1);
    }

    public void transferStop(){
        transfer.setPower(0);
    }


    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (transfer != null) {
            line.addData("Transfer", "pow=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return transfer.getPower();
                }
            });
        }

        if (feeder != null) {
            line.addData("Feeder", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return feeder.getPosition();
                }
            });
        }
    }

}



