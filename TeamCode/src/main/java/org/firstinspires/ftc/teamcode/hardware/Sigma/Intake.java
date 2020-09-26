package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * StoneGrabber spec:
 */
public class Intake extends Logger<Intake> implements Configurable {

    final private CoreSystem core;

    private DcMotor intakeMotor;
    private AdjustableServo rightIntakeDrop;

    private final double INTAKE_FAST = 0.9;
    private final double INTAKE_SPEED = 0.5;

    private final double RIGHT_INTAKE_DROP_INIT = 0.995;
    private final double RIGHT_INTAKE_DROP_DOWN = 0.085;

    private boolean intakeDropDown = false;
    private boolean intakeOn = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "Intake";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public Intake(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (rightIntakeDrop != null)
            intakeDropInit();
        if (intakeMotor != null)
            intakeStop();
    }

    public void configure(Configuration configuration, boolean auto) {

        rightIntakeDrop = new AdjustableServo(0,1).configureLogging(
                logTag + ":rightIntakeDrop", logLevel
        );
        rightIntakeDrop.configure(configuration.getHardwareMap(), "rightIntakeDrop");
        configuration.register(rightIntakeDrop);
        intakeDropInit();

        intakeMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "intakeMotor");
        if (intakeMotor != null) intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intakeDropInit() {
        rightIntakeDrop.setPosition(RIGHT_INTAKE_DROP_INIT);
        intakeDropDown = false;
    }

    public void intakeDropDown(){
        rightIntakeDrop.setPosition(RIGHT_INTAKE_DROP_DOWN);
        intakeDropDown = true;
    }

    public void intakeDropAuto(){
        if (intakeDropDown)
            intakeDropInit();
        else
            intakeDropDown();
    }

    public void intakeStop(){
        intakeMotor.setPower(0.0);
        intakeOn = false;
    }

    public void intakeAuto(boolean fast) {
       if (intakeOn)
           intakeStop();
       else
           intakeIn(fast);
    }

    public void intakeIn(boolean fast){
        intakeOn = true;
        if(fast)
            intakeMotor.setPower(INTAKE_FAST);
        else
            intakeMotor.setPower(INTAKE_SPEED);
    }


    public void intakeOut(boolean fast){
        intakeOn = false;
        if(fast)
            intakeMotor.setPower(-INTAKE_FAST);
        else
            intakeMotor.setPower(-INTAKE_SPEED);
    }


    /*
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (intakeMotor != null) {
            line.addData("intakeMotor", "power=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return intakeMotor.getPower();
                }
            });
        }

        if (rightIntakeDrop != null) {
            line.addData("rightIntakeDrop", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return rightIntakeDrop.getPosition();
                }
            });
        }

    }
    
}