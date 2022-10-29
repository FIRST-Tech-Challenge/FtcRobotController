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
                leftRotationSwitchMotors = leftTurnDirectionW;
                rightRotationSwitchMotors = rightTurnDirectionW;
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
                leftRotatePower = snapLeftWheelPID.update(rightCurrentW);
                rightRotatePower = snapRightWheelPID.update(leftCurrentW);
                spinPower = 0;

                // 8/8 items
                break;

            case STOP: //this is NOT reset
                rightThrottle = 1;
                leftThrottle = 1;
                spinPower = 0;
                leftRotatePower = 0;
                rightRotatePower = 0;
                translationPowerPercentage = 0;
                rotationPowerPercentage = 0;
                rightRotClicks = 0;
                leftRotClicks = 0;
                spinClicks = 0;
                setClicksCycle = false;
                leftRotationSwitchMotors = 1;
                rightRotationSwitchMotors = 1;
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
        return (Math.abs(leftCurrentW - leftOptimizedTargetW) >= constants.degreeTOLERANCE || Math.abs(rightCurrentW - rightOptimizedTargetW) >= constants.degreeTOLERANCE);
    }

    public void setPos(){
        //setting targets
        trackJoystickL();
        double[] leftWheelTargets = wheelOptimization(lx, ly, leftCurrentW);
        double[] rightWheelTargets =  wheelOptimization(lx, ly, rightCurrentW);
        double[] robotTargets = robotHeaderOptimization(rx, ry);

        leftTurnAmountW = leftWheelTargets[0]; //optimized turn amount
        rightTurnAmountW = rightWheelTargets[0];
        leftTargetW = leftWheelTargets[1]; //target orientation for wheel
        rightTargetW = rightWheelTargets[1];

        leftTurnDirectionW = (int)leftWheelTargets[2];
        rightTurnDirectionW = (int)rightWheelTargets[2];

        leftOptimizedTargetW = clamp(leftCurrentW + leftTurnAmountW); //optimized target orientation
        rightOptimizedTargetW = clamp(rightCurrentW + rightTurnAmountW);

        //setting PIDs for rotation of wheels & robot
        snapLeftWheelPID.setTargets(leftOptimizedTargetW, 0.03, 0, 0.01);
        snapRightWheelPID.setTargets(rightOptimizedTargetW, 0.03, 0, 0.01);
//        tableSpinWheelPID.setTargets(targetR, 0.03, 0, 0);
    }

    public double getRTargetW(){
        return rightTargetW;
    }
    public double getLTargetW(){
        return leftTargetW;
    }
    public int getRightDirectionW(){
        return rightTurnDirectionW;
    }
    public int getLeftDirectionW(){
        return leftTurnDirectionW;
    }
    public double getLTurnAmount(){
        return leftTurnAmountW;
    }
    public double getRTurnAmount(){
        return rightTurnAmountW;
    }
    public double getLOptimizedTargetW(){
        return leftOptimizedTargetW;
    }
    public double getROptimizedTargetW(){
        return rightOptimizedTargetW;
    }

    public void getRotTargetClicks(){
        if (setClicksCycle == false){
            setClicksCycle = true;
            leftRotClicks = (int)(leftTurnAmountW * constants.CLICKS_PER_DEGREE * leftTurnDirectionW);
            rightRotClicks = (int)(rightTurnAmountW * constants.CLICKS_PER_DEGREE * rightTurnDirectionW);
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

        motorPower[0] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + leftRotatePower * rotationPowerPercentage * leftRotationSwitchMotors; //top left
        motorPower[1] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + leftRotatePower * rotationPowerPercentage * leftRotationSwitchMotors; //bottom left
        motorPower[2] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rightRotatePower * rotationPowerPercentage * rightRotationSwitchMotors; //top right
        motorPower[3] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rightRotatePower * rotationPowerPercentage * rightRotationSwitchMotors; //bottom right

        return motorPower;
    }

    public int[] getClicks(){
        int[] clicks = new int[4];
        clicks[0] = -spinClicks + leftRotClicks; //left
        clicks[1] = spinClicks + leftRotClicks; //left
        clicks[2] = -spinClicks + rightRotClicks; //right
        clicks[3] = spinClicks  + rightRotClicks; //right
        return clicks;
    }

    public dType getdDriveType(){
        return dtype;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }

}

