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
public class IntakeV2 extends Logger<IntakeV2> implements Configurable {

    final private CoreSystem core;

    private DcMotor rightIntakeMotor;
    private DcMotor leftIntakeMotor;
    private AdjustableServo rightIntakeDrop;

    private final double INTAKE_FAST = 1.0;
    private final double INTAKE_SPEED = 0.5;

    private final double RIGHT_INTAKE_DROP_INIT = 1.0;
    private final double RIGHT_INTAKE_DROP_DOWN = 0.165;

    private boolean intakeDropDown = false;
    private boolean intakeOn = false;
    private int intake_count = 0;

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
    public IntakeV2(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (rightIntakeDrop != null)
            intakeDropInit();
        if (rightIntakeMotor != null || leftIntakeMotor != null)
            intakeStop();
    }

    public void configure(Configuration configuration, boolean auto) {

        rightIntakeDrop = new AdjustableServo(0,1).configureLogging(
                logTag + ":rightIntakeDrop", logLevel
        );
        rightIntakeDrop.configure(configuration.getHardwareMap(), "rightIntakeDrop");
        configuration.register(rightIntakeDrop);
        intakeDropInit();

        rightIntakeMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "rightIntakeMotor");
        if (rightIntakeMotor != null) rightIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftIntakeMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "leftIntakeMotor");
        if (leftIntakeMotor != null) leftIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        rightIntakeMotor.setPower(0.0);
        leftIntakeMotor.setPower(0.0);
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
        intake_count++;
        double ratio = ((intake_count%5)==0?0.5:1.0);
        if(fast){
            rightIntakeMotor.setPower(INTAKE_FAST);
            leftIntakeMotor.setPower(INTAKE_FAST*ratio);
        }
        else{
            rightIntakeMotor.setPower(INTAKE_SPEED);
            leftIntakeMotor.setPower(INTAKE_SPEED);
        }
    }


    public void intakeOut(boolean fast){
        intakeOn = false;
        if(fast){
            rightIntakeMotor.setPower(-INTAKE_FAST);
            leftIntakeMotor.setPower(-INTAKE_FAST);
        }
        else{
            rightIntakeMotor.setPower(-INTAKE_SPEED);
            leftIntakeMotor.setPower(-INTAKE_SPEED);

        }
    }


    /*
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (rightIntakeMotor != null) {
            line.addData("rightIntakeMotor", "power=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return rightIntakeMotor.getPower();
                }
            });
        }

        if (leftIntakeMotor != null) {
            line.addData("rightIntakeMotor", "power=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return leftIntakeMotor.getPower();
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