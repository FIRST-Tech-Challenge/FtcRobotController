package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.auto.Math.SplineMath;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class Kinematics {
    Constants constants = new Constants();
    SplineMath splinemath;
    LinearMath linearmath;

    public enum DriveType{
        LINEAR,
        SNAP,
        SPLINE,
        TURN, //robot turns on its center
        STOP,
        NOT_INITIALIZED
    }
    private DriveType type = DriveType.NOT_INITIALIZED;

    public enum Mode{AUTO, TELEOP};
    private Mode mode = Mode.AUTO;

    //robot's power
    private double rotatePower = 0.0;
    private double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;
    private double leftThrottle = 1.0;
    private double rightThrottle = 1.0;
    private double speed = 0.0; //for autoMode (between 0~1)
    private int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
    private int translateSwitchMotors = 1; //1 if going forward, -1 if going backward

    //current orientation
    GlobalPosSystem posSystem;
    private double currentW; //current wheel orientation
    private double currentR; //current robot header orientation

    //PIDs
    private RotateSwerveModulePID snapWheelPID;
    private RotateSwerveModulePID tableSpinWheelPID;
    private RotateSwerveModulePID resetWheelPID;
    private RotateSwerveModulePID counteractSplinePID;

    //targets
    double x = 0.0;
    double y = 0.0;
    double wheelTurnAmount = 0.0; //how much the wheels should rotate
    double robotTurnAmount = 0.0; //how much the robot should turn
    double targetW = 0.0;
    double prevTargetW = 0.0;
    double targetR = 0.0;
    double splineReference = 0.0;

    //checkover
    public boolean firstMovement = true; //true when robot is stopped.  False while it is moving.

    public Kinematics(GlobalPosSystem posSystem){
        splinemath = new SplineMath();
        linearmath = new LinearMath();
        snapWheelPID = new RotateSwerveModulePID();
        tableSpinWheelPID = new RotateSwerveModulePID();
        resetWheelPID = new RotateSwerveModulePID();
        counteractSplinePID = new RotateSwerveModulePID();
        this.posSystem = posSystem;
    }

    public void logic(){
        switch(type){
            case LINEAR:
                setSpin();
                tableSpin();
                // 7 / 8 items (missing translationPowerPercentage, but we don't set that here)
                firstMovement = false;
                break;

            case SNAP:
                //rotate modules until target is hit
                rightThrottle = 1;
                leftThrottle = 1;
                rotationSwitchMotors = (int)getWheelDirection(x, y)[2];
                translateSwitchMotors = rotationSwitchMotors;
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
                rotatePower = snapWheelPID.update(currentW);
                // 7 / 8 items (missing spinPower, but we don't spin while snapping)
                break;

            case SPLINE:
                setSpin();
                if (Math.abs(currentW - targetW) <= constants.TOLERANCE){
                    double throttle = Math.tanh(Math.abs(y / x));
                    rightThrottle = (rotationSwitchMotors == 1 ? throttle : 1);
                    leftThrottle = (rotationSwitchMotors == 1 ? 1 : throttle);
                    translateSwitchMotors = 1; //robot is always going to spline forward I think
                    if (Math.abs(splineReference - posSystem.getPositionArr()[3]) <= constants.TOLERANCE){
                        rotatePower = counteractSplinePID.update(posSystem.getPositionArr()[3]);
                        rotationSwitchMotors = (int)getRobotDirection(splineReference)[2];
                    }
                    tableSpin();
                } else{
                    type = DriveType.LINEAR;
                }
                // 8/8 items
                break;

            case TURN:
                //...
                setSplineReference();
                break;

            case STOP: //this is NOT reset
                rightThrottle = 0;
                leftThrottle = 0;
                spinPower = 0;
                rotatePower = 0;
                translationPowerPercentage = 0;
                rotationPowerPercentage = 0;

                // 6/8 items (missing switchMotors for rotation and translation, but we don't really need to change that)
                break;
        }
        prevTargetW = targetW;
    }

    public boolean shouldStop(){
        return (Math.abs(targetW - prevTargetW) > 45);
    }

    public boolean shouldSnap(){
        return (Math.abs(currentW - targetW) <= constants.TOLERANCE);
    }

    public boolean canSpline(){
        return (type != DriveType.STOP && type != DriveType.SNAP && type != DriveType.TURN && !firstMovement);
    }


    private void tableSpin(){
        rotatePower = tableSpinWheelPID.update(currentR);
        rotationSwitchMotors = (int)getRobotDirection(targetR)[2];

        if (Math.abs(targetR - currentR) <= constants.TOLERANCE){ //while target not hit
            rotationPowerPercentage = 0.5; //this may or may not be good
            translationPowerPercentage = 0.5;
            if (type != DriveType.SPLINE )setSplineReference();
            /*
            possible alternative:
            rotationPowerPercentage = rotatePower / 1.4;
            translationPowerPercentage = 1 - rotatePower;
             */
        } else{ //after target is hit, stop table spinning
            rotationPowerPercentage = 0.0;
            translationPowerPercentage = 1.0;
        }
    }


    public void setPos(DriveType type, double x, double y, double robotTurnAmount, double speed){
        this.type = type;
        this.x = x;
        this.y = y;
        this.speed = speed;

        //setting targets
        this.wheelTurnAmount = getWheelDirection(x, y)[0];
        this.robotTurnAmount = robotTurnAmount; //how much the robot heading should turn
        targetW = clamp(currentW + wheelTurnAmount);
        targetR = clamp(currentR + robotTurnAmount); //the target orientation is on a circle of (-179, 180]

        //setting PIDs for rotation of wheels & robot
        snapWheelPID.setTargets(targetW, 0, 0, 0);
        tableSpinWheelPID.setTargets(targetR, 0, 0, 0);

        //specifically for auto (not teleop)
        splinemath.setInits(posSystem.getMotorClicks()[2], posSystem.getMotorClicks()[0]);
        splinemath.setPos(x, y, robotTurnAmount);
        linearmath.setInits(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
        linearmath.setPos(x, y, robotTurnAmount);
    }

    public void setSpin(){
        if (mode == Mode.TELEOP) {
            spinPower = Math.sqrt(Math.pow(x,2) + Math.pow(y, 2));
            rightThrottle = 1;
            leftThrottle = 1;
        }
        else {
            switch (type){
                case SPLINE:
                    spinPower = 1;
                    rightThrottle = splinemath.returnPowerR(posSystem.getMotorClicks()[2]);
                    leftThrottle = splinemath.returnPowerL(posSystem.getMotorClicks()[0]);
                    //NOTE: May need to apply the powers to the spin power instead of the throttles (and make it spinPowerR, spinPowerL)
                    break;

                case LINEAR:
                    spinPower = linearmath.getSpinPower(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
                    break;
            }
        }
    }

    public void setCurrents(){
        currentW = posSystem.getPositionArr()[2];
        currentR = posSystem.getPositionArr()[3];
    }

    private void setSplineReference(){
        splineReference = posSystem.getPositionArr()[3];
        counteractSplinePID.setTargets(splineReference, 0, 0, 0);
    }

    public void setMode(Mode mode){
        this.mode = mode;
    } //Important to set this at the "init" for teleop and auto

    public double[] getWheelDirection(double x, double y){ //returns how much the wheels should rotate in which direction
        double[] directionArr = new double[3];

        //determine targets
        double target =  Math.toDegrees(Math.atan2(x, y)); //finds target orientation in terms of degrees (range is (-180, 180])
        directionArr[1] = target;

        //determine how much modules must turn in which direction (optimization)
        double turnAmount = target - currentW;
        double switchMotors = Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180){
            turnAmount = 360 - Math.abs(turnAmount);
            switchMotors *= -1;
        }
        if(Math.abs(turnAmount) > 90){
            target += 180;
            target = clamp(target);
            turnAmount = target - currentW;
            switchMotors *= -1;
            if(Math.abs(turnAmount) > 180){
                turnAmount = 360 - Math.abs(turnAmount);
                switchMotors *= -1;
            }
        }
        directionArr[0] = turnAmount;
        directionArr[2] = switchMotors; //1 = right, -1 = left

        return directionArr;
    }

    public double[] getRobotDirection(double x, double y){
        double[] directionArr = new double[3];

        //determine targets
        double target = Math.toDegrees(Math.atan2(x, y));
        directionArr[1] = target;

        //determine how much robot header must turn in which direction
        double turnAmount = target-currentR;
        double switchMotors = Math.signum(turnAmount); //pos = right, neg = left
        if(Math.abs(turnAmount) > 180) {
            turnAmount = 360 - Math.abs(turnAmount);
            switchMotors *= -1;
        }
        directionArr[0] = Math.abs(turnAmount);
        directionArr[2] = switchMotors;

        return directionArr;
    }

    public double[] getRobotDirection(double targetOrientation){
        double[] directionArr = new double[3];

        //determine targets
        directionArr[1] = targetOrientation;

        //determine how much robot header must turn in which direction
        double turnAmount = targetOrientation-currentR;
        double switchMotors = Math.signum(turnAmount);
        if(Math.abs(turnAmount) > 180) {
            turnAmount = 360 - Math.abs(turnAmount);
            switchMotors *= -1;
        }
        directionArr[0] = Math.abs(turnAmount);
        directionArr[2] = switchMotors;

        return directionArr;
    }

    public DriveType getDriveType(){
        return type;
    }

    public double[] getVelocity(){
        double[] motorPower = new double[4];

        motorPower[0] = constants.MAX_VELOCITY_DT * speed * (spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top left
        motorPower[1] = constants.MAX_VELOCITY_DT * speed * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom left
        motorPower[2] = constants.MAX_VELOCITY_DT * speed * (spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top right
        motorPower[3] = constants.MAX_VELOCITY_DT * speed * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rightThrottle * rotationPowerPercentage * rotationSwitchMotors); //bottom right

        return motorPower;
    }

    public double[] getPower(){
        double[] motorPower = new double[4];

        motorPower[0] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top left
        motorPower[1] = -1 * spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom left
        motorPower[2] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top right
        motorPower[3] = -1 * spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom right

        return motorPower;
    }

    public int[] getSplineClicks(){
        int[] clicks = new int[4];
        for (int i = 0; i < 4; i++){
            clicks[i] = splinemath.getClicks()[i];
        }
        return clicks;
    }

    public int[] getLinearClicks(){
        int[] clicks = new int[4];
        for (int i = 0; i < 4; i++){
            clicks[i] = linearmath.getClicks()[i];
        }
        return clicks;
    }

    public double clamp(double degrees){
        if (degrees < -179 || degrees > 180) degrees %= (-1 * Math.signum(degrees) * 360);
        return degrees;
    }

    public boolean resetStuff(){
        if (currentW != 0){
            rotationSwitchMotors = (currentW > 0 ? -1 : 1); //1 = right, -1 = left

            resetWheelPID.setTargets(0, 0, 0, 0);
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
        firstMovement = true;

        return (currentW == 0);
    }
}

