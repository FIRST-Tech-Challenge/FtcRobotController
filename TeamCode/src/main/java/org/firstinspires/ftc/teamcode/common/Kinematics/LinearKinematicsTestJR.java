package org.firstinspires.ftc.teamcode.common.Kinematics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

public class LinearKinematicsTestJR extends Kinematics{
    ElapsedTime timer = new ElapsedTime();

    private double lx;
    private double ly;
    private double rx;
    private double ry;

    private double joystickL = 0.0;
    private double prevJoystickL = 0.0;
    private int joystickCount = 0;

    private boolean firstMovement = false; //true when robot is stopped.  False while it is moving.
    private boolean setClicksCycle = false;
    public boolean inCycle = false;

    public enum dType{
        SNAP,
        STOP,
        NOT_INITIALIZED
    }
    public dType dtype = dType.NOT_INITIALIZED;

    public LinearKinematicsTestJR(GlobalPosSystem posSystem) {
        super(posSystem); //runs Kinematics constructor
    }

    public void logic(){
        inCycle = (Math.abs(joystickL - prevJoystickL) <= 5);
        if (!inCycle || noMovementRequests()) dtype = dType.STOP;
        else dtype = dType.SNAP;

        switch(dtype){
            case SNAP:
                getRotTargetClicks();
                //rotate modules until target is hit
                rightThrottle = 1;
                leftThrottle = 1;
                translateSwitchMotors = turnDirectionW; //this is wrong
                rotationSwitchMotors = turnDirectionW;
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
                rotatePower = snapWheelPID.update(currentW);
                spinPower = 0;

                // 8/8 items
                break;

            case STOP: //this is NOT reset
                rightThrottle = 1;
                leftThrottle = 1;
                spinPower = 0;
                rotatePower = 0;
                translationPowerPercentage = 0;
                rotationPowerPercentage = 0;
                rotClicks = 0;
                spinClicks = 0;
                setClicksCycle = true;
                rotationSwitchMotors = 1;
                translateSwitchMotors = 1;

                // 6/8 items (missing switchMotors for rotation and translation, but we don't really need to change that)
                break;

            default:
                type = DriveType.STOP;
                break;
        }

        firstMovement = noMovementRequests();
    }

    public boolean shouldSnap(){
        return (Math.abs(currentW - optimizedTargetW) >= constants.degreeTOLERANCE);
    }

    public void setPos(){
        //setting targets
        trackJoystickL();
        double[] wheelTargets = wheelOptimization(lx, ly);
        double[] robotTargets = robotHeaderOptimization(rx, ry);

        turnAmountW = wheelTargets[0]; //optimized turn amount
        targetW = wheelTargets[1]; //target orientation for wheel
        turnDirectionW = (int)wheelTargets[2];

        optimizedTargetW = clamp(currentW + turnAmountW); //optimized target orientation

        //setting PIDs for rotation of wheels & robot
        snapWheelPID.setTargets(optimizedTargetW, 0.03, 0, 0);
//        tableSpinWheelPID.setTargets(targetR, 0.03, 0, 0);
    }

    public double getTargetW(){
        return targetW;
    }
    public int getDirectionW(){
        return turnDirectionW;
    }
    public double getTurnAmount(){
        return turnAmountW;
    }
    public double getOptimizedTargetW(){
        return optimizedTargetW;
    }



    public void getRotTargetClicks(){
        if (setClicksCycle == false){
            setClicksCycle = true;
            rotClicks = (int)(turnAmountW * constants.CLICKS_PER_DEGREE * turnDirectionW);
        } else if (!shouldSnap()){
            setClicksCycle = false;
        }
        spinClicks = 0;
    }

    private void trackJoystickL(){
        joystickCount++;
        if(joystickCount > 4){
            joystickCount = 0;
            prevJoystickL = joystickL;
            joystickL = Math.toDegrees(Math.atan2(lx, ly));
        }
    }

    public void getGamepad(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
    }

    public double[] getPower(){
        double[] motorPower = new double[4];

        motorPower[0] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top left
        motorPower[1] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom left
        motorPower[2] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top right
        motorPower[3] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom right

        return motorPower;
    }

    public int[] getClicks(){
        int[] clicks = new int[4];
        clicks[0] = -spinClicks + rotClicks;
        clicks[1] = spinClicks + rotClicks;
        clicks[2] = -spinClicks + rotClicks;
        clicks[3] = spinClicks  + rotClicks;
        return clicks;
    }

    public dType getdDriveType(){
        return dtype;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }

}

