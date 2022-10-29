package org.firstinspires.ftc.teamcode.common.Kinematics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

public class LinearKinematicsTest extends Kinematics{
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
        LINEAR,
        SNAP,
        STOP,
        NOT_INITIALIZED
    }
    public dType dtype = LinearKinematicsTest.dType.NOT_INITIALIZED;

    public LinearKinematicsTest(GlobalPosSystem posSystem) {
        super(posSystem); //runs Kinematics constructor
    }

    public void logic(){
        inCycle = (Math.abs(joystickL - prevJoystickL) <= 10);
        if (!inCycle || noMovementRequests()) dtype = dType.STOP;
        else if (shouldSnap()) dtype = dType.SNAP;
        else dtype = dType.LINEAR;

        switch(dtype){
            case LINEAR:
                spinPower = Math.sqrt(Math.pow(lx,2) + Math.pow(ly, 2));
                rotatePower = 0;
                if (Math.abs(spinPower) >= 1) spinPower = 1;
                rightThrottle = 1;
                leftThrottle = 1;
                translationPowerPercentage = 1;
                rotationPowerPercentage = 0;

                getSpinTargetClicks();

                // 7 / 8 items (missing translationPowerPercentage, but we don't set that here)
                break;

            case SNAP:
                getRotTargetClicks(turnAmountW, turnDirectionW);
                //rotate modules until target is hit
                rightThrottle = 1;
                leftThrottle = 1;
                translateSwitchMotors = (int)wheelOptimization(lx, ly)[2];
                rotationSwitchMotors = translateSwitchMotors;
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
        snapWheelPID.setTargets(optimizedTargetW, 0.01, 0, 0);
        tableSpinWheelPID.setTargets(targetR, 0.025, 0, 0);
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



    public void getRotTargetClicks(double turnAmount, int direction){
        if (setClicksCycle == false){
            setClicksCycle = true;
            rotClicks = (int)(turnAmount * constants.CLICKS_PER_DEGREE * direction);
        } else if (!shouldSnap()){
            setClicksCycle = false;
        }
        spinClicks = 0;
    }

    public void getSpinTargetClicks(){
        spinClicks = (int)(100 * spinPower * translationPowerPercentage);
        rotClicks = 0;
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

