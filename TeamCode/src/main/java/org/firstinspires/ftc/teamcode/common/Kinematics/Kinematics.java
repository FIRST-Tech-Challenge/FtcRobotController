package org.firstinspires.ftc.teamcode.common.Kinematics;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;

public class Kinematics {
    protected Constants constants = new Constants();

    public enum DriveType{
        LINEAR,
        SNAP,
        SPLINE,
        TURN, //robot turns on its center
        STOP,
        NOT_INITIALIZED
    }
    protected DriveType type = DriveType.NOT_INITIALIZED;

    //robot's power
    protected double rotatePower = 0.0;
    protected double spinPower = 0.0;
    protected double translationPowerPercentage = 0.0;
    protected double rotationPowerPercentage = 0.0;
    protected double leftThrottle = 1.0;
    protected double rightThrottle = 1.0;
    protected int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
    protected int translateSwitchMotors = 1; //1 if going forward, -1 if going backward

    //current orientation
    protected GlobalPosSystem posSystem;
    protected double currentW; //current wheel orientation
    protected double currentR; //current robot header orientation

    //PIDs
    protected RotateSwerveModulePID snapWheelPID;
    protected RotateSwerveModulePID tableSpinWheelPID;
    protected RotateSwerveModulePID resetWheelPID;
    protected RotateSwerveModulePID counteractSplinePID;

    //targets
    protected double targetW = 0.0;
    protected double optimizedTargetW = 0.0;
    protected double targetR = 0.0;
    protected double splineReference = 0.0;

    public Kinematics(GlobalPosSystem posSystem){
        snapWheelPID = new RotateSwerveModulePID();
        tableSpinWheelPID = new RotateSwerveModulePID();
        resetWheelPID = new RotateSwerveModulePID();
        counteractSplinePID = new RotateSwerveModulePID();
        this.posSystem = posSystem;
    }

    public void setCurrents(){
        currentW = posSystem.getPositionArr()[2];
        currentR = posSystem.getPositionArr()[3];
    }

    public void setSplineReference(){
        splineReference = posSystem.getPositionArr()[3];
        counteractSplinePID.setTargets(splineReference, 0.05, 0, 0.02);
    }

    public double[] wheelOptimization(double x, double y){ //returns how much the wheels should rotate in which direction
        double[] directionArr = new double[3];

        //determine targets
        double target = (y==0 ? (90 * Math.signum(x)) : Math.toDegrees(Math.atan2(x, y)));
        directionArr[1] = target;

        //determine how much modules must turn in which direction (optimization)
        double turnAmount = target - currentW;
        double turnDirection = Math.signum(turnAmount);
        double switchMotors = Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180){
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }
        if(Math.abs(turnAmount) > 90){
            target += 180;
            target = clamp(target);
            turnAmount = target - currentW;
            switchMotors = -1;
            if(Math.abs(turnAmount) > 180){
                turnAmount = 360 - Math.abs(turnAmount);
            }
        }
        directionArr[0] = (turnAmount * turnDirection);
        directionArr[2] = switchMotors;

        return directionArr;
    }

    public double[] robotHeaderOptimization(double x, double y){
        double[] directionArr = new double[2];

        //determine targets
        double target = (y==0 ? (45 * Math.signum(x)) : Math.toDegrees(Math.atan2(x, y)));
        directionArr[1] = target;

        //determine how much robot header must turn in which direction
        double turnAmount = target-currentR;
        double turnDirection = Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180) {
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }
        directionArr[0] = (Math.abs(turnAmount) * turnDirection);


        return directionArr;
    }

    public double robotHeaderOptimization(double targetOrientation){
        //determine how much robot header must turn in which direction
        double turnAmount = targetOrientation-currentR;
        double turnDirection = Math.signum(turnAmount);
        if(Math.abs(turnAmount) > 180) {
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }
        return (turnAmount * turnDirection);
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }

    public DriveType getDriveType(){
        return type;
    }

    public boolean resetStuff(){
        if (currentW != 0){
            rotationSwitchMotors = (currentW > 0 ? -1 : 1); //1 = right, -1 = left

            resetWheelPID.setTargets(0, 0.03, 0, 0);
            rotatePower = resetWheelPID.update(currentW);

            translationPowerPercentage = 0.0;
            rotationPowerPercentage = 1.0;
            leftThrottle = 1.0;
            rightThrottle = 1.0;
            spinPower = 0.0;
            translateSwitchMotors = 1;
        } else{
            translationPowerPercentage = 0.0;
            rotationPowerPercentage = 0.0;
            rotatePower = 0.0;
            rotationSwitchMotors = 1;
        }
        // firstMovement = true;

        return (currentW == 0);
    }
}

