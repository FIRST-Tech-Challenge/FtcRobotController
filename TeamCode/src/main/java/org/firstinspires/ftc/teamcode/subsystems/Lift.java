package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class Lift extends Subsystem {
    RobotConfig r;

    private DcMotorEx lift0;
    private DcMotorEx lift1;
    private static double liftPos;

    private double previousTime;
    private double previousError;
    
    private double P;
    private double I; //needs to be given a value of 0 at the start
    private double D;
    private double targetPos; //needs to be set whenever you need it to be changed
    private double spoolHoldPosition = 0;
    private boolean spoolHold = false;

    double liftPositioner;
    boolean limitIsPressed;
    PoleHeights targetHeight = PoleHeights.GROUND;


    public Lift(RobotConfig r) {
        this.r = r;
    }
    public Lift(){
        this(RobotConfig.getInstance());
    }

    private void resetEncoders(){
        lift0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init() {
        lift0 = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.lift0);
        lift1 = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.lift1);
        previousTime = 0;
        previousError = 0;
        targetPos = 0;
        spoolHoldPosition = 0;
        spoolHold = false;
    }

    @Override
    public void read() {
        double lift0Pos = lift0.getCurrentPosition();
        double lift1Pos = lift1.getCurrentPosition();

        if(((lift0Pos + lift1Pos) / 2) < -1000) liftPos = Math.min(lift0Pos, lift1Pos);
        else if(((lift0Pos + lift1Pos) / 2) >= -1000) liftPos = Math.max(lift0Pos, lift1Pos);
        else liftPos = 0;
    
        r.setDelivery(liftPos < -500);
    }

    double cachedPower;
    
    @Override
    public void update(){
        double currentTime = RobotConfig.elapsedTime.time();
        
        if(limitIsPressed) {
            resetEncoders();
        }

        positionControl(liftPositioner, limitIsPressed, targetHeight);

        double currentError = (targetPos - liftPos);
        final double kP = 0.005;
        final double kI = -0.5;
        final double kD = 0.0;

        P = kP*currentError;
        I = (liftPos/ PoleHeights.HIGH.getEncoderValue()) * kI;
        D = kD*((currentError - previousError) / (currentTime - previousTime));
        
        
        if(Math.abs(Math.abs(PID()) - Math.abs(cachedPower)) >= 0.01) {
            lift0.setPower(PID());
            lift1.setPower(PID());
            cachedPower = PID();
        }
        
        previousError = currentError;
        previousTime = currentTime;
    }

    @Override
    public void close() {

    }

    private void positionControl(double liftPosition, boolean limitIsPressed, PoleHeights poleHeight){
        targetPos -= ((int) (liftPosition*250));
        if (Double.isNaN(liftPosition) || liftPosition == 0.0 && !spoolHold){
            spoolHoldPosition = Lift.liftPos;
            spoolHold = true;
        }
        if(liftPosition == 0.0 || Double.isNaN(liftPosition)){
            targetPos = spoolHoldPosition;
        }
        else {
            spoolHold = false;
        }
        switch (poleHeight){
            case HIGH:
                targetPos = PoleHeights.HIGH.getEncoderValue() + 450;
                spoolHoldPosition = PoleHeights.HIGH_DROP.getEncoderValue() + 300;
                break;
            case MEDIUM:
                targetPos = PoleHeights.MEDIUM.getEncoderValue() + 450;
                spoolHoldPosition = PoleHeights.MEDIUM_DROP.getEncoderValue() + 300;
                break;
            case LOW:
                targetPos = PoleHeights.LOW.getEncoderValue() + 450;
                spoolHoldPosition = PoleHeights.LOW_DROP.getEncoderValue() + 300;
                break;
            case GROUND:
                targetPos = PoleHeights.GROUND.getEncoderValue();
                spoolHoldPosition = PoleHeights.GROUND.getEncoderValue();
                break;
            case STACK4:
                targetPos = PoleHeights.STACK4.getEncoderValue();
                spoolHoldPosition = PoleHeights.STACK4.getEncoderValue();
                break;
            case STACK3:
                targetPos = PoleHeights.STACK3.getEncoderValue();
                spoolHoldPosition = PoleHeights.STACK3.getEncoderValue();
                break;
            case STACK2:
                targetPos = PoleHeights.STACK2.getEncoderValue();
                spoolHoldPosition = PoleHeights.STACK2.getEncoderValue();
                break;
            case STACK1:
                targetPos = PoleHeights.STACK1.getEncoderValue();
                spoolHoldPosition = PoleHeights.STACK1.getEncoderValue();
                break;
            case STACK0:
                targetPos = PoleHeights.STACK0.getEncoderValue();
                spoolHoldPosition = PoleHeights.STACK0.getEncoderValue();
                break;
            case IDLE:
                break;
            default:
                break;
        }
        
        if((targetPos > Lift.liftPos) && limitIsPressed){
            targetPos = Lift.liftPos;
        }
        if (targetPos > 100){
            targetPos = 100;
        }
        if(targetPos < (PoleHeights.HIGH.getEncoderValue())) {
            targetPos = (PoleHeights.HIGH.getEncoderValue());
        }
    }

    private PoleHeights buttonAnalysis(boolean top, boolean middle, boolean low, boolean ground){
        if (top) return PoleHeights.HIGH;
        if (middle) return PoleHeights.MEDIUM;
        if (low) return PoleHeights.LOW;
        if (ground) return PoleHeights.GROUND;
        return PoleHeights.IDLE;
    }

    public void liftInputs(double liftPositionInput, boolean limitSwitch, boolean top, boolean middle, boolean low, boolean ground){
        liftPositioner = liftPositionInput;
        limitIsPressed = limitSwitch;
        targetHeight = buttonAnalysis(top, middle, low, ground);
    }
    
    public void presetLiftPosition(PoleHeights poleHeight){
        targetHeight = poleHeight;
        liftPositioner = 0;
    }
    
    public void limitSwitchInput(boolean limitIsPressed){
        this.limitIsPressed = limitIsPressed;
    }
    
    private double PID(){
        return P+I+D;
    }
    
    public enum PoleHeights {
        HIGH(-2750),
        MEDIUM(-1950),
        LOW(-1250),
        GROUND(0),
        HIGH_DROP(-2350),
        MEDIUM_DROP(-1750),
        LOW_DROP(-1000),
        STACK4(-410),
        STACK3(-320),
        STACK2(-230),
        STACK1(-70),
        STACK0(0),
        IDLE(0);

        PoleHeights(int encoderValue){
            this.encoderValue = encoderValue;
        }

        private final int encoderValue;

        public int getEncoderValue() {
            return encoderValue;
        }
    }
}
