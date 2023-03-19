package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Lift extends Subsystem {
    RobotConfig r;

    private static DcMotorEx lift0;
    private static DcMotorEx lift1;
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
    RobotConstants.poleHeights targetHeight;


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
    
        r.delivery = liftPos < -500;
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
        I = (liftPos/RobotConstants.poleHeights.HIGH.getEncoderValue()) * kI;
        D = kD*((currentError - previousError) / (currentTime - previousTime));
        
        
        if(Math.abs(Math.abs(PID()) - Math.abs(cachedPower)) >= 0.01) {
            lift0.setPower(PID());
            lift1.setPower(PID());
            cachedPower = PID();
        }
        
        r.opMode.telemetry.addData("targetPosition", targetPos);
        r.opMode.telemetry.addData("position", liftPos);

        previousError = currentError;
        previousTime = currentTime;
    }

    @Override
    public void close() {

    }

    private void positionControl(double liftPosition, boolean limitIsPressed, RobotConstants.poleHeights poleHeight){
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
                targetPos = RobotConstants.poleHeights.HIGH.getEncoderValue() + 200;
                spoolHoldPosition = RobotConstants.poleHeights.HIGH_DROP.getEncoderValue() + 200;
                break;
            case MEDIUM:
                targetPos = RobotConstants.poleHeights.MEDIUM.getEncoderValue() + 200;
                spoolHoldPosition = RobotConstants.poleHeights.MEDIUM_DROP.getEncoderValue() + 200;
                break;
            case LOW:
                targetPos = RobotConstants.poleHeights.LOW.getEncoderValue() + 200;
                spoolHoldPosition = RobotConstants.poleHeights.LOW_DROP.getEncoderValue() + 200;
                break;
            case GROUND:
                targetPos = RobotConstants.poleHeights.GROUND.getEncoderValue();
                spoolHoldPosition = RobotConstants.poleHeights.GROUND.getEncoderValue();
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
        if(targetPos < (RobotConstants.poleHeights.HIGH.getEncoderValue())) {
            targetPos = (RobotConstants.poleHeights.HIGH.getEncoderValue());
        }
        
        
    }

    private RobotConstants.poleHeights buttonAnalysis(boolean top, boolean middle, boolean low, boolean ground){
        if (top) return RobotConstants.poleHeights.HIGH;
        if (middle) return RobotConstants.poleHeights.MEDIUM;
        if (low) return RobotConstants.poleHeights.LOW;
        if (ground) return RobotConstants.poleHeights.GROUND;
        return RobotConstants.poleHeights.IDLE;
    }

    public void liftInputs(double liftPositionInput, boolean limitSwitch, boolean top, boolean middle, boolean low, boolean ground){
        liftPositioner = liftPositionInput;
        limitIsPressed = limitSwitch;
        targetHeight = buttonAnalysis(top, middle, low, ground);
    }
    
    private double PID(){
        return P+I+D;
    }
}
