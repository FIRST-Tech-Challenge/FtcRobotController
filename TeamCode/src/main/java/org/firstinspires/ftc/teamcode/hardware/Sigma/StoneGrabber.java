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
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import static java.lang.Thread.currentThread;

/**
 * StoneGrabber spec:
 */
public class StoneGrabber extends Logger<StoneGrabber> implements Configurable {

    final private CoreSystem core;

    private DcMotor leftLifter;
    private DcMotor rightLifter;
    private AdjustableServo arm;
    private AdjustableServo wrist;
    private AdjustableServo grabber;
    private AdjustableServo capstoneServo;
    private ElapsedTime SGTimer = new ElapsedTime();
    private double waitSec;

    private final double ARM_OFFSET = .0; // must > -.04 and < .04)
    private final double ARM_UP = 0.06+ARM_OFFSET;
    private final double ARM_READY_GRAB = 0.99+ARM_OFFSET;
    private final double ARM_DOWN = 0.86+ARM_OFFSET;  // right position to grab stone inside
    private final double ARM_DOWN_MORE = ARM_DOWN+ARM_OFFSET+0.06;  // right position to grab stone inside
    private final double ARM_DOWN_MORE_CAP = ARM_DOWN+ARM_OFFSET+0.09;  // right position to grab stone inside
    private final double ARM_DOWN_SAFE = 0.86+ARM_OFFSET;
    private final double ARM_DOWN_WITH_STONE = 0.90+ARM_OFFSET;
    // private final double ARM_INITIAL = 0.84+ARM_OFFSET;
    private final double ARM_INITIAL = ARM_READY_GRAB;
    private final double ARM_OUT_INIT = 0.47+ARM_OFFSET;
    private final double ARM_IN = 0.65+ARM_OFFSET;
    private final double ARM_LOW = 0.58+ARM_OFFSET;
    private final double ARM_OUT = 0.35+ARM_OFFSET;
    private final double ARM_OUT_MORE = 0.27+ARM_OFFSET;
    private final double ARM_OUT_AUTO = 0.45+ARM_OFFSET;
    private final double ARM_DOWN_FOR_CAP = 0.72+ARM_OFFSET;
    private final double ARM_CAPSTONE = 0.78+ARM_OFFSET;
    private final double ARM_CAPSTONE_MORE = 0.9+ARM_OFFSET;
    private final double ARM_DELIVER_LOW = 0.35+ARM_OFFSET;
    private final double ARM_DELIVER = 0.28+ARM_OFFSET;
    private final double ARM_DELIVER_HIGHER = 0.26+ARM_OFFSET;
    private final double ARM_DELIVER_THROW = 0.14+ARM_OFFSET;
    private final double ARM_MIN = 0.12+ARM_OFFSET;

    private final double ARM_INC_UNIT = 0.02;

    private final double WRIST_PARALLEL = 0.66;
    private final double WRIST_PERPENDICULAR = 0.08;
    private final double WRIST_INIT = WRIST_PERPENDICULAR;
    private final double WRIST_INC_UNIT = 0.01;
    private final double WRIST_CAPSTONE = WRIST_PARALLEL - 0.01;

    private final double GRABBER_CLOSE = 0.29;
    private final double GRABBER_OPEN_IN = 0.59;
    private final double GRABBER_INIT = GRABBER_OPEN_IN;
    private final double GRABBER_VERTICAL = 0.5;
    private final double GRABBER_OPEN = 0.9;
    private final double GRABBER_OPEN_AUTO = 0.98;


    private final int LIFT_THRESHOLD = 15;
    private final int LIFT_RUN_TO_POSITION_OFFSET = 100;  // V5.3, new control for goBilda 5205 motor
    private final int LIFT_DOWN_GRAB = 0;
    private final int LIFT_DOWN = 0;
    private final int LIFT_DOWN_GRAB_INSIDE = 150;
    private final int LIFT_GRAB_READY_CAPSTONE = 310;
    private final int LIFT_GRAB = 180;
    private final int LIFT_GRAB_AUTO = 20;
    private final int LIFT_MIN = 0;
    private final int LIFT_MAX = 3200;
    private final int LIFT_SAFE_SWING_AUTO = 500;
    private final int LIFT_SAFE_SWING = 500;
    private final int LIFT_SAFE_BRIDGE = 540;
    private final int LIFT_SAFE_SWING_IN = 600;
    private final int LIFT_SAFE_DELIVERY = 250;
    private final int LIFT_UP_FOR_REGRAB = 215;
    private final int LIFT_UP_FOR_CAP = 640;
    private final int LIFT_UP_BEFORE_CAP = 500;
    private final int LIFT_UP_FINAL_CAP = 1050;
    //private final double LIFT_POWER = 0.5;   // V5.2
    private final double LIFT_POWER = 1.0;  // V5.3
    private final double LIFT_POWER_DOWN = 0.8;
    private final double LIFT_POWER_COMBO = 0.6;
    private final double LIFT_POWER_HOLD = 0.3;
    private final double LIFT_POWER_SLOW = 0.6;
    private final double LIFT_POWER_SLOW_DOWN = 0.3;
    private final int LIFT_DELIVER = 1000;

    private final double CAPSTONE_INIT = 0.0;
    private final double CAPSTONE_OUT = 0.6;



    private boolean armIsDown = false;
    private boolean armIsIn = true;
    private boolean isGrabberOpened = false;
    private boolean isWristParallel = false;
    private boolean isCapstoneServoOut = false;
    private ElapsedTime runtime = new ElapsedTime();
    private int lift_target_pos;


    @Override
    public String getUniqueName() {
        return "StoneGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public StoneGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto, boolean armOut) {
        if (arm != null)
            armInit(armOut);
        if (wrist!=null) {
            if (armOut || Auto)
                wristParallel();
            else {
                wristParallel();
                // wristInit();
            }
        }
        if (grabber!=null)
            grabberInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        arm = new AdjustableServo(0,1).configureLogging(
                logTag + ":arm", logLevel
        );
        arm.configure(configuration.getHardwareMap(), "arm");
        configuration.register(arm);
        // armInit(false);

        wrist = new AdjustableServo(0,1).configureLogging(
                logTag + ":wrist", logLevel
        );
        wrist.configure(configuration.getHardwareMap(), "wrist");
        configuration.register(wrist);
        // wristInit();

        grabber = new AdjustableServo(0,1).configureLogging(
                logTag + ":grabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);
        grabberInit();

        capstoneServo = new AdjustableServo(0,1).configureLogging(
                logTag + ":capstoneServo", logLevel
        );
        capstoneServo.configure(configuration.getHardwareMap(), "capstoneServo");
        configuration.register(capstoneServo);
        capstoneServoInit();

        leftLifter = configuration.getHardwareMap().tryGet(DcMotor.class, "leftLifter");
        //if (leftLifter != null) leftLifter.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftLifter != null) {
            leftLifter.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // register hanging as configurable component
        // configuration.register(this);

        rightLifter = configuration.getHardwareMap().tryGet(DcMotor.class, "rightLifter");
        if (rightLifter != null) {
            // rightLifter.setDirection(DcMotorSimple.Direction.REVERSE);
            rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
    }

    public boolean isArmInside() {
        return armIsIn;
    }

    public boolean isArmDown() {
        return armIsDown;
    }

    public void armInit(boolean armOut) {
        arm.setPosition((armOut?ARM_OUT_INIT:ARM_INITIAL));
        armIsDown = false;
        armIsIn = !armOut;
    }

    public void armOut() {
        arm.setPosition(ARM_OUT);
        armIsDown = false;
        armIsIn = false;
    }

    public void armDownInc() {
        double cur_pos = arm.getPosition();
        cur_pos += ARM_INC_UNIT;
        if (cur_pos>1) cur_pos=1;
        arm.setPosition(cur_pos);
        armIsDown = false;
        if (cur_pos>ARM_IN)
            armIsIn = true;
        else
            armIsIn = false;
        if (Math.abs(cur_pos-ARM_DOWN)<0.2)
            armIsDown = true;
    }

    public void armUpInc(boolean force) {
        double cur_pos = arm.getPosition();
        cur_pos -= ARM_INC_UNIT;
        if ((cur_pos<ARM_MIN) && !force)
            cur_pos=ARM_MIN;
        if (cur_pos<0) cur_pos=0;
        arm.setPosition(cur_pos);
        armIsDown = false;
        if (cur_pos<ARM_LOW)
            armIsIn = false;
        else
            armIsIn = true;
        if (Math.abs(cur_pos-ARM_DOWN)<0.2)
            armIsDown = true;
    }

    public void armUp() {
        arm.setPosition(ARM_UP);
        armIsDown = false;
        armIsIn = false;
    }

    public void armDown() {
        arm.setPosition(ARM_DOWN);
        armIsDown = true;
        armIsIn = true;
    }

    public void armDownSafe() {
        arm.setPosition(ARM_DOWN_SAFE);
        armIsDown = true;
        armIsIn = true;
    }

    public void armDeliver() {
        arm.setPosition(ARM_DELIVER);
        armIsDown = false;
        armIsIn = false;
    }

    public void armAuto() {
        if (armIsDown) {
            armUp();
        } else {
            armDown();
        }
    }

    public void wristInit() {
        if (wrist==null) return;
        wrist.setPosition(WRIST_INIT);
        isWristParallel = false;
    }

    public void wristParallel () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PARALLEL);
        isWristParallel = true;
    }

    public void wristPerpendicular () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PERPENDICULAR);
        isWristParallel = false;
    }

    public void wristAuto() {
        if (isWristParallel) wristPerpendicular();
        else wristParallel();
    }

    public void wristLeftInc() {
        double cur_pos = wrist.getPosition();
        cur_pos -= WRIST_INC_UNIT;
        if (cur_pos<0) cur_pos=0;
        wrist.setPosition(cur_pos);
        if (cur_pos<=(WRIST_PARALLEL+0.2))
            isWristParallel = true;
        else
            isWristParallel = false;

    }
    public void wristRightInc() {
        double cur_pos = wrist.getPosition();
        cur_pos += WRIST_INC_UNIT;
        if (cur_pos>1) cur_pos=1;
        wrist.setPosition(cur_pos);
        if (cur_pos>=(WRIST_PERPENDICULAR-0.2))
            isWristParallel = false;
        else
            isWristParallel = true;

    }

    public Progress moveWrist(boolean parallel) {
        double target = parallel ? WRIST_PARALLEL : WRIST_PERPENDICULAR;
        isWristParallel = parallel;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveWrist(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from parallel to vertical takes 2 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(500 * adjustment);
        wrist.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }
    public Progress moveWristForCapstone() {
        double target = WRIST_CAPSTONE;
        isWristParallel = true;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveWrist(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from parallel to vertical takes 2 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(500 * adjustment);
        wrist.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void grabberInit() {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_INIT);
        isGrabberOpened = true;
    }

    public void grabberOpen () {
        if (grabber==null) return;
        if (armIsIn)
            grabber.setPosition(GRABBER_OPEN_IN);
        else
            grabber.setPosition(GRABBER_OPEN);
        isGrabberOpened = true;
    }
    public void grabberOpenAuto () {
        grabber.setPosition(GRABBER_OPEN_AUTO);
        isGrabberOpened = true;
    }

    public void grabberReGrab () {
        if (grabber==null) return;
        grabberOpen();
        for(int i=0; i<1000; i++) // add some dummy delay
            grabber.getPosition();
        grabberClose();
    }

    public void grabberClose () {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_CLOSE);
        isGrabberOpened = false;
    }

    public void grabberAuto() {
        if (isGrabberOpened)
            grabberClose();
        else
            grabberOpen();
    }

    public Progress moveGrabberPos(double target) {
        isGrabberOpened = (target>GRABBER_VERTICAL+0.05);
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveGrabber(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from up to down takes 1 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(500 * adjustment);
        grabber.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public Progress moveGrabber(boolean closed) {
        double target = GRABBER_CLOSE;
        if (!closed) {
           if (armIsIn)
               target = GRABBER_OPEN_IN;
           else
               target = GRABBER_OPEN;
        }
        isGrabberOpened = !closed;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveGrabber(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from up to down takes 1 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(500 * adjustment);
        grabber.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void capstoneLeftInc() {
        double cur_pos = capstoneServo.getPosition();
        cur_pos -= WRIST_INC_UNIT;
        if (cur_pos<0) cur_pos=0;
        capstoneServo.setPosition(cur_pos);
        if (Math.abs(cur_pos-CAPSTONE_OUT)<0.2)
            isCapstoneServoOut = true;
        else
            isCapstoneServoOut = false;
    }

    public void capstoneRightInc() {
        double cur_pos = capstoneServo.getPosition();
        cur_pos += WRIST_INC_UNIT;
        if (cur_pos>1) cur_pos=1.0;
        capstoneServo.setPosition(cur_pos);
        if (Math.abs(cur_pos-CAPSTONE_OUT)<0.2)
            isCapstoneServoOut = true;
        else
            isCapstoneServoOut = false;
    }

    public void capstoneServoInit() {
        capstoneServo.setPosition(CAPSTONE_INIT);
        isCapstoneServoOut = false;
    }

    public void capstoneServoOut(){
        capstoneServo.setPosition(CAPSTONE_OUT);
        isCapstoneServoOut = true;
    }

    public void capstoneServoAuto(){
        if(isCapstoneServoOut)
            capstoneServoInit();
        else
            capstoneServoOut();
    }

    public void liftResetEncoder() {
        leftLifter.setPower(0);
        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLifter.setPower(0);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

//    public void liftToPosition(int pos, double power) {
//        if (leftLifter==null) return;
//        if (Math.abs(leftLifter.getCurrentPosition()-pos)<20) return;
//
//        leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftLifter.setTargetPosition(pos);
//        leftLifter.setPower(power);
//    }

    public void liftUp (boolean slow, boolean force)  {
        if (leftLifter == null || rightLifter == null) return;
        // leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (!force) {
            if (leftLifter.getCurrentPosition() > LIFT_MAX || rightLifter.getCurrentPosition() > LIFT_MAX) {
                liftStop();
                liftHold();
                return;
            }
        }
        if (slow || force){
            leftLifter.setPower(LIFT_POWER_SLOW);
            rightLifter.setPower(LIFT_POWER_SLOW);
        }
        else{
            leftLifter.setPower(LIFT_POWER);
            rightLifter.setPower(LIFT_POWER);
        }
    }

    public void liftDown(boolean slow, boolean force) {
        if (leftLifter ==null || rightLifter == null) return;
        // leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!force) {
            int bufferOffset = (slow?0:100);
            if (leftLifter.getCurrentPosition() < LIFT_MIN + bufferOffset|| rightLifter.getCurrentPosition() < LIFT_MIN + bufferOffset) {
                liftStop();
                return;
            }
        }
        if (slow || force){
            leftLifter.setPower(-LIFT_POWER_SLOW_DOWN);
            rightLifter.setPower(-LIFT_POWER_SLOW_DOWN);
        }
        else{
            leftLifter.setPower(-LIFT_POWER_DOWN);
            rightLifter.setPower(-LIFT_POWER_DOWN);
        }
    }

    public void liftStop() {
        if (leftLifter ==null || rightLifter == null) return;
        // leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLifter.setPower(0);
        rightLifter.setPower(0);
        //int pos = leftLifter.getCurrentPosition();
        // liftToPosition(pos, true);
        // leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftLifter.setTargetPosition(pos);
        // leftLifter.setPower(LIFT_POWER_HOLD);
    }
    public void liftHold() {
        if (leftLifter ==null || rightLifter == null) return;
        // leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLifter.setPower(0);
        rightLifter.setPower(0);
        int posL = leftLifter.getCurrentPosition();
        int posR = rightLifter.getCurrentPosition();
        //int posL = rightLifter.getCurrentPosition();
        // liftToPosition(pos, true);
        leftLifter.setTargetPosition(posL);
        rightLifter.setTargetPosition(posR);
        leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLifter.setPower(LIFT_POWER_HOLD);
        rightLifter.setPower(LIFT_POWER_HOLD);

//        rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightLifter.setTargetPosition(posL);
//        rightLifter.setPower(LIFT_POWER_HOLD);
    }

    public void liftToSafe() {
        liftToPosition(LIFT_SAFE_SWING, false);
    }

    public Progress liftToPosition(int pos, boolean slow) {
        if (leftLifter ==null || rightLifter == null) return null;
        // if (Math.abs(leftLifter.getCurrentPosition()-pos)<20) return null;

        leftLifter.setPower(0);
        rightLifter.setPower(0);
        leftLifter.setTargetPosition(pos);
        leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLifter.setTargetPosition(pos);
        rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_target_pos = pos;

        leftLifter.setPower((slow ? LIFT_POWER_SLOW : LIFT_POWER_COMBO));
        rightLifter.setPower((slow ? LIFT_POWER_SLOW : LIFT_POWER_COMBO));

        return new Progress() {
            public boolean isDone() {
                int lpos = leftLifter.getCurrentPosition();
                if (Math.abs(leftLifter.getCurrentPosition()-lift_target_pos)<LIFT_RUN_TO_POSITION_OFFSET) {
                    if (lpos>200) {
                        leftLifter.setPower(0);
                        rightLifter.setPower(0);
                        liftHold();
                    }
                    return true;
                }
                return !leftLifter.isBusy();
            }
        };
    }

    public Progress liftToPositionQuick(int pos, boolean slow) {
        if (leftLifter ==null || rightLifter == null) return null;
        // if (Math.abs(leftLifter.getCurrentPosition()-pos)<20) return null;

        leftLifter.setPower(0);
        rightLifter.setPower(0);
        // leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftLifter.setTargetPosition(pos);
        // leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_target_pos = pos;
        //if (Math.abs(leftLifter.getCurrentPosition()-lift_target_pos)<LIFT_THRESHOLD || Math.abs(rightLifter.getCurrentPosition()-lift_target_pos)<LIFT_THRESHOLD){
        if (Math.abs(leftLifter.getCurrentPosition()-lift_target_pos)<LIFT_THRESHOLD){
            leftLifter.setPower(0);// liftStop();
            rightLifter.setPower(0);
        }
        else if (leftLifter.getCurrentPosition()<pos){
            leftLifter.setPower((slow?LIFT_POWER_SLOW:LIFT_POWER_COMBO));
            rightLifter.setPower((slow?LIFT_POWER_SLOW:LIFT_POWER_COMBO));
        }
        else{
            leftLifter.setPower((slow?-LIFT_POWER_SLOW:-LIFT_POWER_COMBO));
            rightLifter.setPower((slow?-LIFT_POWER_SLOW:-LIFT_POWER_COMBO));
        }
        return new Progress() {
            public boolean isDone() {
                int lpos = leftLifter.getCurrentPosition();
                if (Math.abs(lpos-lift_target_pos)<LIFT_THRESHOLD) {
                    leftLifter.setPower(0);
                    rightLifter.setPower(0);
                    liftHold();
                    return true;
                } else if ((lpos<lift_target_pos && leftLifter.getPower()<0)||
                        lpos>lift_target_pos && leftLifter.getPower()>0) { // over shoot
                    leftLifter.setPower(0);
                    rightLifter.setPower(0);
                    liftHold();
                    return true;
                } else if (lpos>LIFT_MAX-200||lpos<(LIFT_MIN-10)) {
                    leftLifter.setPower(0);
                    rightLifter.setPower(0);
                    liftHold();
                    return true;
                }
                return false;
                // return !leftLifter.isBusy() || Math.abs(leftLifter.getTargetPosition() - leftLifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
            }
        };
    }

    private Progress moveArm(double position) {
        double adjustment = Math.abs(position - arm.getPosition());
        arm.setPosition(position);
        if (position>=ARM_IN)
            armIsIn=true;
        else
            armIsIn=false;
        if (Math.abs(position-ARM_DOWN)<0.2)
            armIsDown = true;
        else
            armIsDown = false;
        // 3.3ms per degree of rotation
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 600);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void armOutComboAuto() {
        if (currentThread().isInterrupted()) return;
        armOutCombo();
        while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }
    }

    public void waitTM(double sec) {
        final String taskName = "Wait";
        if (!TaskManager.isComplete(taskName)) return;
        waitSec = sec;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                SGTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                };
            }
        }, taskName);
    }
    public void armOutCombo(){
        armOutCombo(0, true);
    }
    public void armOutCombo(double delaySec, boolean auto) {
        final String taskName = "Arm Out Combo";
        if (!TaskManager.isComplete(taskName)) return;
        boolean grabIsOpened = isGrabberOpened;
        boolean armWasOut = !isArmInside();
        if (delaySec>0) {
            waitSec = delaySec;
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    SGTimer.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                    };
                }
            }, taskName);
        }
        if (!armWasOut) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN_SAFE);
                }
            }, taskName);
            if (leftLifter.getCurrentPosition()<LIFT_SAFE_SWING) {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        if (isWristParallel && isGrabberOpened) grabberClose();
                        return liftToPosition(LIFT_SAFE_SWING, false);
                    }
                }, taskName);
            }
        }
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT_AUTO);
                }
            }, taskName);
        } else if (armWasOut){
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT_MORE);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT);
                }
            }, taskName);
        }
        if (leftLifter.getCurrentPosition()<=LIFT_SAFE_SWING) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_GRAB, false);
                }
            }, taskName);
        }
        if (grabIsOpened) { // restore grabber to open
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(false);
                }
            }, taskName);
        }
    }
    public void armOutReadyGrabAutoCombo() {
        final String taskName = "Arm Out Auto Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpenAuto();
                return moveArm(ARM_OUT_AUTO);
            }
            }, taskName);

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftToPosition(LIFT_GRAB_AUTO, false);
            }
        }, taskName);
    }

    public void grabInsideAndArmOutCombo(double delaySec, boolean auto) {
        final String taskName = "Grab Inside and Arm Out Combo";
        if (!TaskManager.isComplete(taskName)) return;
        boolean armWasOut = !isArmInside();
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                armDown();
                return liftToPosition(LIFT_DOWN, false);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(true);
            }
        }, taskName);
        if (delaySec>0) {
            waitSec = delaySec;
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    SGTimer.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                    };
                }
            }, taskName);
        }
        if (!armWasOut) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN_SAFE);
                }
            }, taskName);
            if (leftLifter.getCurrentPosition()<LIFT_SAFE_SWING) {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        if (isWristParallel && isGrabberOpened) grabberClose();
                        return liftToPosition(LIFT_SAFE_SWING+150, false);
                    }
                }, taskName);
            }
        }
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT_AUTO);
                }
            }, taskName);
        }
    }
    public void releaseStoneCombo() {
        final String taskName = "Release Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(false);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {

                int cur_pos = leftLifter.getCurrentPosition();
                int target_pos = cur_pos + 300;
                if(target_pos > LIFT_MAX)
                    target_pos = LIFT_MAX;
                return liftToPosition(target_pos, true);
            }
        }, taskName);
    }
    public void armInComboAuto(final boolean wristParallel) {
        if (currentThread().isInterrupted()) return;
        armInCombo(wristParallel, true);
        while (!TaskManager.isComplete("Arm In Combo")) {
            TaskManager.processTasks();
        }
    }
    public void armInCombo(final boolean wristParallel, boolean isAuto) {
        final String taskName = "Arm In Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(true);
            }
        }, taskName);
        if (wristParallel!=isWristParallel) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    moveWrist(wristParallel);
                    return moveGrabber(true);
                }
            }, taskName);
        }
        if (leftLifter.getCurrentPosition()<LIFT_SAFE_SWING) {
            if (isAuto) {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        return liftToPosition(LIFT_SAFE_SWING_AUTO, false);
                    }
                }, taskName);
            } else {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        return liftToPosition(LIFT_SAFE_SWING, false);
                    }
                }, taskName);
            }
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_WITH_STONE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                return liftToPosition(LIFT_DOWN, true);
            }
        }, taskName);

        waitSec = 0.3;
        TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    grabberOpen();
                    SGTimer.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                    }; }
                    }, taskName);

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                return new Progress() {
                    @Override
                    public boolean isDone() { return true; }
                };
            }
        }, taskName);
    }

    public void grabStoneComboAuto() {
        if (currentThread().isInterrupted()) return;
        grabStoneCombo();
        while (!TaskManager.isComplete("Grab Stone Combo")) {
            TaskManager.processTasks();
        }
    }
    public void grabStoneCombo() { // grab stone from outside used by auto
        final String taskName = "Grab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftToPosition(LIFT_DOWN_GRAB, true);
            }
        }, taskName);
        if (arm.getPosition() < ARM_OUT_AUTO) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT_AUTO);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress grabberProgress = moveGrabber(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return grabberProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                arm.setPosition(ARM_DELIVER);
                return liftToPosition(LIFT_DOWN, false);
            }
        }, taskName);

    }

    public void grabCapStoneCombo() {
        final String taskName = "Grab Cap Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        boolean isLiftUp = (leftLifter.getCurrentPosition()>=LIFT_GRAB);
        if (!isLiftUp) { // stage-1
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(false); // open grabber
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_GRAB_READY_CAPSTONE,false);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_CAPSTONE);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(true); // close grabber
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    capstoneServoOut();
                    moveWristForCapstone();
                    return moveArm(ARM_CAPSTONE);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_UP_BEFORE_CAP, true);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_CAPSTONE_MORE);
                }
            }, taskName);
        }
        // else { // stage-2
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_UP_FINAL_CAP, true);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    capstoneServoInit();
                    return moveWrist(true);
                }
            }, taskName);
        // }
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return liftToPosition(LIFT_GRAB, false);
//            }
//        }, taskName);
//
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveArm(ARM_DOWN_FOR_CAP);
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveGrabber(false); // open grabber
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveArm(ARM_DOWN_SAFE);
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return liftToPosition(LIFT_DOWN_GRAB+50, false);
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveGrabber(true); // close grabber
//            }
//        }, taskName);
    }

    public void grabStoneInsideCombo() {
        final String taskName = "Grab Stone Inside Combo";
        if (!TaskManager.isComplete(taskName)) return;

        // armInReadyGrabCombo();
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                // armDown();
                return liftToPosition(LIFT_DOWN_GRAB_INSIDE, true);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                armDown();
                return liftToPosition(LIFT_DOWN, false);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(true);
            }
        }, taskName);
    }

    public void regrabStoneCombo(boolean moreIn) {
        final String taskName = "Regrab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return true;
                    }
                };
            }
        }, taskName);
        if (!moreIn) { // move out a little bit
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_GRAB, false);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN_FOR_CAP);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN_MORE);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(false); // open grabber
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftToPosition(LIFT_DOWN_GRAB, false);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(true); // close grabber
            }
        }, taskName);
    }

    public void armInReadyGrabCombo() {
        final String taskName = "Arm In Ready Grab Combo";
        if (!TaskManager.isComplete(taskName)) return;
        final boolean armWasIn = armIsIn;
        if (!armWasIn) { // move arm inside
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabberPos(GRABBER_VERTICAL);
                }
            }, taskName);
            if (leftLifter.getCurrentPosition()<LIFT_SAFE_SWING) {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        int position = LIFT_SAFE_SWING;
                        return liftToPosition(position, false);
                    }
                }, taskName);
            }
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    grabberOpen();
                    int position = LIFT_DOWN_GRAB;
                    return liftToPosition(position, false);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress wristProgress = moveWrist(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return wristProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                return moveArm(ARM_READY_GRAB);
            }
        }, taskName);
    }

    public void armInLiftUpReadyGrabCombo() {
        final String taskName = "Arm In LiftUp Ready Grab Combo";
        if (!TaskManager.isComplete(taskName)) return;
        final boolean armWasIn = armIsIn;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                int position = LIFT_SAFE_SWING_IN;
                if (armWasIn)
                    position = LIFT_SAFE_BRIDGE;
                return liftToPosition(position, false);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress wristProgress = moveWrist(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return wristProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                return moveArm(ARM_DOWN);
            }
        }, taskName);
        if (!armWasIn) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    int position = LIFT_SAFE_BRIDGE;
                    return liftToPosition(position, false);
                }
            }, taskName);
        }
    }

    public void deliverStoneComboAuto() {
        if (currentThread().isInterrupted()) return;
        deliverStoneCombo(true);
        while (!TaskManager.isComplete("Deliver Stone Combo")) {
            TaskManager.processTasks();
        }
    }

    public void deliverStoneCombo(boolean auto) {
        final String taskName = "Deliver Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DELIVER_LOW);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT);
                }
            }, taskName);
        }
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    grabberOpen();
                    return liftToPosition(LIFT_SAFE_DELIVERY, false);
                }
            }, taskName);
        }
    }

    public void deliverStoneThrowComboAuto() {
        if (currentThread().isInterrupted()) return;
        deliverStoneThrowCombo();
        while (!TaskManager.isComplete("Deliver Stone Throw Combo")) {
            TaskManager.processTasks();
        }
    }

    public void deliverStoneThrowCombo() {
        final String taskName = "Deliver Stone Throw Combo";
        if (!TaskManager.isComplete(taskName)) return;

            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DELIVER);
                }
            }, taskName);

            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_SAFE_SWING-400, false);
                }
            }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                return moveArm(ARM_DELIVER_THROW);
            }
        }, taskName);
    }
    public void lifterDownCombo() {
        lifterDownCombo(0.0);
    }
    public void lifterDownCombo(double delaySec) {
        final String taskName = "Lifter Down Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (delaySec>0) {
            waitSec = delaySec;
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    SGTimer.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                    };
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftToPosition(LIFT_DOWN, false);
            }
        }, taskName);
    }
    /*
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (arm != null) {
            line.addData("arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return arm.getPosition();
                }
            });
            line.addData("/armIn", "=%s", new Func<String>() {
                @Override
                public String value() {
                    return ((armIsIn?"T":"F"));
                }
            });
        }
        if (wrist != null) {
            line.addData("wrist", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return wrist.getPosition();
                }
            });
        }
        if (grabber != null) {
            line.addData("grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }
        if (leftLifter != null) {
            line.addData("leftLifter", "pos=%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return leftLifter.getCurrentPosition();
                }
            });
        }
        if (rightLifter != null) {
            line.addData("rightLifter", "pos=%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return rightLifter.getCurrentPosition();
                }
            });
        }

    }
    public void grabStoneComboHigher(){
        final String taskName = "Grab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftToPosition(LIFT_DOWN_GRAB,true);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_OUT_AUTO);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress grabberProgress = moveGrabber(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return grabberProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                arm.setPosition(ARM_DELIVER_HIGHER);
                return liftToPosition(LIFT_DOWN+800,false);
            }
        }, taskName);
    }

    public void grabStoneComboAutoHigher() {
        if (currentThread().isInterrupted()) return;
        grabStoneComboHigher();
        while (!TaskManager.isComplete("Grab Stone Combo")) {
            TaskManager.processTasks();
        }
    }
    public void deliverAndArmIn(){
        deliverStoneCombo(true);
        armInCombo(true, true);
    }

}



