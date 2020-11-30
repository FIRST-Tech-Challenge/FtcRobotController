package org.firstinspires.ftc.teamcode.hardware.MechBot;

import android.hardware.camera2.params.TonemapCurve;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

import static java.lang.Thread.sleep;

/**
 * FoundationHook spec:
 */
public class Hopper extends Logger<Hopper> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo feeder;
    private CRServo ringLifter;
    /*private*/ public TouchSensor magTouch;
    public DistanceSensor rangetouch;


    private final double FEEDER_IN = 0.5;
    private final double FEEDER_INIT = FEEDER_IN;
    private final double FEEDER_OUT = 0.9;

    private boolean feederIsIn = true;
    private boolean transferIsDown = true;
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
        if (ringLifter != null)
            servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        ringLifter = configuration.getHardwareMap().get(CRServo.class, "ringLifter");

        feeder = new AdjustableServo(0, 1).configureLogging(
                logTag + ":hopper", logLevel
        );
        feeder.configure(configuration.getHardwareMap(), "feeder");
        magTouch = configuration.getHardwareMap().get(TouchSensor.class, "touch");

        configuration.register(feeder);
        // servoInit();
      // configuration.register(this);
    }

    public void servoInit() {
        ringLifter.setPower(0);
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

    public void feederAuto() throws InterruptedException {
        feederOut();
        sleep(300);
        feederIn();
    }

    public void transferUp(){
        ringLifter.setPower(-1);
    }

    public void transferDown(){
        ringLifter.setPower(1);
    }

    public void transferStop(){
        ringLifter.setPower(0);
    }

    public void transferUpAuto() throws InterruptedException {
        if (ringLifter == null) return;
        if (!transferIsDown) return;
        ringLifter.setPower(-1);
        sleep(300);
        ringLifter.setPower(0);
        transferIsDown = false;
    }

    public void transferDownAuto() throws InterruptedException {
        if (ringLifter == null) return;
        if (transferIsDown) return;
        double iniTime = System.currentTimeMillis();
        ringLifter.setPower(1);
        while(!magTouch.isPressed() && System.currentTimeMillis() - iniTime < 1000 )
            sleep(5);
        ringLifter.setPower(0);
        transferIsDown = true;
    }


    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (ringLifter != null) {
            line.addData("Transfer", "pow=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return ringLifter.getPower();
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

        if (magTouch != null) {
            line.addData("Touch", "pressed=%s", new Func<String>() {
                @Override
                public String value() {
                    return (magTouch.isPressed()?"Yes":"No");
                }
            });
        }
    }

}



