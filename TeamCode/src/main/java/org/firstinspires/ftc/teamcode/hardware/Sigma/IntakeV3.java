package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * StoneGrabber spec:
 */
public class IntakeV3 extends Logger<IntakeV3> implements Configurable {

    final private CoreSystem core;

    private DcMotor rightIntakeMotor;
    private DcMotor leftIntakeMotor;
    private AdjustableServo rightIntakeDrop;
    private CRServo leftIntakeDrop;
    private DigitalChannel prox = null;

    private final double INTAKE_FAST = 1.0;
    private final double INTAKE_SPEED = 0.5;

    private final double RIGHT_INTAKE_DROP_INIT = 0.925;
    private final double RIGHT_INTAKE_DROP_DOWN = 0.06;

    private boolean intakeDropDown = false;
    private boolean intakeOn = false;
    private int intake_count = 0;

    private AdjustableServo frontGate;

    private final double FRONT_GATE_OPEN = 0.85;
    private final double FRONT_GATE_INIT = FRONT_GATE_OPEN;
    private final double FRONT_GATE_CLOSE = 0.26;
    private final double FRONT_GATE_PUSH = 0.04;

    private boolean isGateOpen = true;
    private boolean feederMode = false;

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

    public boolean feederModeCheck(){
        return feederMode;
    }

    public void feederModeAuto(){
        feederMode = !feederMode;
    }

    public IntakeV3(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (rightIntakeDrop != null)
            intakeDropInit();
        if (rightIntakeMotor != null || leftIntakeMotor != null)
            intakeStop();

        if(frontGate != null){
            ingateInit();
        }
    }

    public void configure(Configuration configuration, boolean auto) {

        frontGate = new AdjustableServo(0,1).configureLogging(
                logTag + ":frontGate", logLevel
        );
        frontGate.configure(configuration.getHardwareMap(), "frontGate");
        configuration.register(frontGate);


        rightIntakeDrop = new AdjustableServo(0,1).configureLogging(
                logTag + ":rightIntakeDrop", logLevel
        );
        rightIntakeDrop.configure(configuration.getHardwareMap(), "rightIntakeDrop");
        configuration.register(rightIntakeDrop);

        leftIntakeDrop = configuration.getHardwareMap().tryGet(CRServo.class, "leftIntakeDrop");
        intakeDropInit();

        rightIntakeMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "rightIntakeMotor");
        if (rightIntakeMotor != null) {
            rightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        leftIntakeMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "leftIntakeMotor");
        if (leftIntakeMotor != null) {
            leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        prox = configuration.getHardwareMap().get(DigitalChannel.class, "prox");
        prox.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean proxDetectStone() {
        if (prox==null)
            return true; // assume detecting mineral
        return !prox.getState();
    }

    public void intakeDropInit() {
        rightIntakeDrop.setPosition(RIGHT_INTAKE_DROP_INIT);
        intakeDropDown = false;
        if (leftIntakeDrop!=null) {
            leftIntakeDrop.setPower(0);
        }
    }

    public void ingateInit(){
        frontGate.setPosition(FRONT_GATE_INIT);
        isGateOpen = true;
    }

    public void ingateOpen(){
        frontGate.setPosition(FRONT_GATE_OPEN);
        isGateOpen = true;
    }

    public void ingatePush(){
        frontGate.setPosition(FRONT_GATE_PUSH);
        isGateOpen = false;
    }

    public void ingateClose(){
        intakeStop();
        frontGate.setPosition(FRONT_GATE_CLOSE);
        isGateOpen = false;
    }

    public void ingateAuto(){
        if(isGateOpen)
            ingateClose();
        else
            ingateOpen();
    }

    public void intakeDropDown(){
        rightIntakeDrop.setPosition(RIGHT_INTAKE_DROP_DOWN);
        intakeDropDown = true;
    }
    private Progress moveIntake(double position) {
        double adjustment = Math.abs(position - rightIntakeDrop.getPosition());
        rightIntakeDrop.setPosition(position);
        // 3.3ms per degree of rotation
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 1000);
        if (position<RIGHT_INTAKE_DROP_DOWN+0.1)
            intakeDropDown = true;
        else
            intakeDropDown = false;
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }
    public void intakeInOutCombo() {
        final String taskName = "intake IoOut Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                double pos = (intakeDropDown?RIGHT_INTAKE_DROP_INIT:RIGHT_INTAKE_DROP_DOWN);
                if (intakeDropDown)
                    leftIntakeDropIn();
                else
                    leftIntakeDropOut();
                return moveIntake(pos);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                leftIntakeStop();
                return new Progress() {
                    @Override
                    public boolean isDone() { return true;
                    }
                };
            }
        }, taskName);
    }
    public void leftIntakeDropIn() {
        if (leftIntakeDrop==null) return;
        leftIntakeDrop.setPower(1.0);
    }
    public void leftIntakeDropOut() {
        if (leftIntakeDrop==null) return;
        leftIntakeDrop.setPower(-1.0);
    }
    public void leftIntakeStop() {
        if (leftIntakeDrop==null) return;
        leftIntakeDrop.setPower(0);
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
    private Progress moveGate(double position) {
        double adjustment = Math.abs(position - frontGate.getPosition());
        frontGate.setPosition(position);
        if (position>= FRONT_GATE_OPEN -0.1)
            isGateOpen=true;
        else {
            isGateOpen = false;
            intakeStop();
        }
        // 3.3ms per degree of rotation
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 600);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void inTakeInCombo() {
        final String taskName = "Intake In Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (!isGateOpen) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGate(FRONT_GATE_OPEN);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                if (isGateOpen) {
                    rightIntakeMotor.setPower(INTAKE_FAST);
                    leftIntakeMotor.setPower(INTAKE_FAST);
                }
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return true;
                    }
                };
            }
        }, taskName);
    }

    public void intakeIn(boolean fast){
        intakeOn = true;
        intake_count++;
        double ratio = ((intake_count%5)==0?1.0:1.0);
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

        if(prox != null){
            line.addData("prox", "stoneDetect=%s", new Func<String>() {
                @Override
                public String value() {
                    return (proxDetectStone()?"YES":"NO");
                }
            });
        }

    }
    
}