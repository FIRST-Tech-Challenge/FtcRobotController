package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
public class Shooter extends Logger<Shooter>  {

    final private CoreSystem core;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private final double shooterSpeedFast = 1.0;
    private final double shooterSpeedSlow = 0.3;
    private boolean shooterOn = false;

    public String getUniqueName() {
        return "shooter";
    }

    /**
     * Hanging constructor
     */
    public Shooter(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (shooter1 != null)
            init();
    }

    public void configure(Configuration configuration, boolean auto) {
        shooter1 = configuration.getHardwareMap().get(DcMotorEx.class, "shooter1");
        shooter2 = configuration.getHardwareMap().get(DcMotorEx.class, "shooter2");
        init();
      // configuration.register(this);
    }

    public void stop() {
        shooter1.setPower(0);
        shooter2.setPower(0);
        shooterOn = false;
    }

    public void init() {
        stop();
    }

    public void shootOutFast(){
        shooter1.setPower(shooterSpeedFast);
        shooter2.setPower(shooterSpeedFast);
        shooterOn = true;
    }

    public void shootOutSlow(){
        shooter1.setPower(shooterSpeedSlow);
        shooter2.setPower(shooterSpeedSlow);
        shooterOn = true;
    }
    public void shootAutoFast(){
        if(shooterOn)
            stop();
        else
            shootOutFast();
    }

    public void shootAutoSlow(){
        if(shooterOn)
            stop();
        else
            shootOutSlow();
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (shooter1 != null) {
            line.addData("shooter1", "pow=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return shooter1.getPower();
                }
            });
        }

        if (shooter2 != null) {
            line.addData("shooter2", "pow=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return shooter2.getPower();
                }
            });
        }
    }

}



