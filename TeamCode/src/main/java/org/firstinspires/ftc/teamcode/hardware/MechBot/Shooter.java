package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private final double shooterSpeedFast = 1.0;
    private final double shooterSpeedSlow = 0.3;
    private final int SHOOT_MAX = 2600;
    private final int SHOOT_FAST = 2100;
    private final int SHOOT_MIN = 100;
    private final int SHOOT_INC_STEP = 50;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private int shooterSpeed = SHOOT_FAST;
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
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        // shooter2 = configuration.getHardwareMap().get(DcMotorEx.class, "shooter2");
        init();
      // configuration.register(this);
    }

    public void stop() {
        if (shooter1!=null)
            shooter1.setPower(0);
        if (shooter2!=null)
            shooter2.setPower(0);
        shooterOn = false;
    }

    public void init() {
        stop();
    }

    public void shootOutFast(){
        if (shooter1!=null)
            shooter1.setVelocity(shooterSpeed);
        if (shooter2!=null)
            shooter2.setVelocity(shooterSpeed);
        shooterOn = true;
    }

    public void shootOutByRpm(double rpm){
        if (shooter1!=null)
            shooter1.setVelocity(rpm);
        if (shooter2!=null)
            shooter2.setVelocity(rpm);
        shooterOn = true;
    }

    public void shootOutSlow(){
        if (shooter1!=null)
            shooter1.setPower(shooterSpeedSlow);
        if (shooter2!=null)
            shooter2.setPower(shooterSpeedSlow);
        shooterOn = true;
    }

    public void shootSpeedInc() {
        shooterSpeed+=SHOOT_INC_STEP;
        if (shooterSpeed>SHOOT_MAX)
            shooterSpeed=SHOOT_MAX;
        shootOutFast();
    }

    public void shootSpeedDec() {
        shooterSpeed-=SHOOT_INC_STEP;
        if (shooterSpeed<SHOOT_MIN)
            shooterSpeed=SHOOT_MIN;
        shootOutFast();
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
            line.addData("shooter1", "%s", new Func<String>() {
                @Override
                public String value() {
                    return String.format("speed=%.0f/%d, pwd=%.2f", shooter1.getVelocity(), shooterSpeed,shooter1.getPower());
                }
            });
        }

        if (shooter2 != null) {
            line.addData("shooter2", "%s", new Func<String>() {
                @Override
                public String value() {
                    return String.format("speed=%.0f, pwd=%.2f", shooter1.getVelocity(), shooter1.getPower());
                }
            });
        }
    }

}



