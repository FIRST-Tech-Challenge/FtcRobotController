package org.firstinspires.ftc.teamcode.common.Kinematics;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

public class LinearKinematicsTest extends Kinematics {
    ElapsedTime accelerationTimer = new ElapsedTime();
    boolean isAccelerateCycle = false;

    private double lx;
    private double ly;
    private double rx;
    private double ry;

//    private double joystickL = 0.0;
//    private double prevJoystickL = 0.0;
//    private int joystickCount = 0;

    private boolean firstSnap = false; //true when robot is stopped.  False while it is moving.

    public enum dType{
        LINEAR,
        SNAP,
        STOP,
        NOT_INITIALIZED
    }
    dType dtype = dType.NOT_INITIALIZED;

    public LinearKinematicsTest(GlobalPosSystem posSystem) {
        super(posSystem); //runs Kinematics constructor
        accelerationTimer.reset();
    }

    public void logic(){
        if (noMovementRequests()) dtype = dType.STOP;
        else if (shouldSnap()) dtype = dType.SNAP;
        else dtype = dType.LINEAR;

        switch(dtype){
            case LINEAR:
                spinPower = Math.sqrt(Math.pow(lx,2) + Math.pow(ly, 2));
                rightRotatePower = 0;
                leftRotatePower = 0;

                rightThrottle = 1;
                leftThrottle = 1;

                translationPowerPercentage = 1;
                rotationPowerPercentage = 0;

                getSpinTargetClicks();

                // 7 / 8 items (missing translationPowerPercentage, but we don't set that here)
                break;
            case SNAP:
                if (firstSnap){
                    firstSnap = false;
                }
                getRotTargetClicks();
                //rotate modules until target is hit
                rightThrottle = 1;
                leftThrottle = 1;
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
                leftRotatePower = snapLeftWheelPID.update(rightCurrentW);
                rightRotatePower = snapRightWheelPID.update(leftCurrentW);
                spinPower = 0;

                // 8/8 items
                break;

            case STOP: //this is NOT reset
                stop();
                break;

            default:
                type = DriveType.STOP;
                break;
        }
    }

    public boolean shouldSnap(){
        return (Math.abs(leftCurrentW - leftOptimizedTargetW) >= constants.degreeTOLERANCE || Math.abs(rightCurrentW - rightOptimizedTargetW) >= constants.degreeTOLERANCE);
    }

    public void stop(){
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

        leftTurnAmountW = 0;
        rightTurnAmountW = 0;

        isAccelerateCycle = false;
        // 6/8 items (missing switchMotors for translation, but we should not change that)
    }

    public void setPos(){
        //setting targets
        //trackJoystickL();
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
        leftRotClicks = (int)(leftTurnAmountW * constants.CLICKS_PER_DEGREE * leftTurnDirectionW);
        rightRotClicks = (int)(rightTurnAmountW * constants.CLICKS_PER_DEGREE * rightTurnDirectionW);

        spinClicks = 0;
    }

    public void getSpinTargetClicks(){
        spinClicks = (int)(100 * spinPower * translationPowerPercentage * translateSwitchMotors);
        rightRotClicks = 0;
        leftRotClicks = 0;
    }
//
//    private void trackJoystickL(){
//        joystickCount++;
//        if(joystickCount >= 3){
//            joystickCount = 0;
//            prevJoystickL = joystickL;
//            joystickL = Math.toDegrees(Math.atan2(lx, ly));
//        }
//    }

    public void getGamepad(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
    }

    public double[] getPower(){
        double[] motorPower = new double[4];

        motorPower[0] = spinPower * translationPowerPercentage * leftThrottle + leftRotatePower * rotationPowerPercentage; //top left
        motorPower[1] = spinPower * translationPowerPercentage * leftThrottle + leftRotatePower * rotationPowerPercentage; //bottom left
        motorPower[2] = spinPower * translationPowerPercentage * rightThrottle + rightRotatePower * rotationPowerPercentage; //top right
        motorPower[3] = spinPower * translationPowerPercentage * rightThrottle + rightRotatePower * rotationPowerPercentage; //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator(motorPower[i]);
        }

        return motorPower;
    }

    public double accelerator(double power){
        if (power == 0) return 0.0;

        if (!isAccelerateCycle){
            accelerationTimer.reset();
            isAccelerateCycle = true;
        }
        double accelerationFactor = (Math.tanh(0.5 * accelerationTimer.milliseconds() - 1.5) / 2.5) + 0.6;
        power *= accelerationFactor;

        if (power > 1) power = 1;
        else if (power < -1) power = -1;

        return power;
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

