package org.firstinspires.ftc.teamcode.common.Kinematics;


import org.firstinspires.ftc.teamcode.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.auto.Math.SplineMath;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

public class AutoKinematics extends Kinematics{
    SplineMath splinemath;
    LinearMath linearmath;

    double x = 0.0;
    double y = 0.0;
    double robotTurnAmount = 0.0; //how much the robot should turn
    double speed = 0.0; //for autoMode (between 0~1)


    public AutoKinematics(GlobalPosSystem posSystem) {
        super(posSystem);
    }

    public void setPos(double x, double y, double robotTurnAmount, double speed){
        this.x = x;
        this.y = y;
        this.speed = speed;


        //setting targets
        double targetWOrientation = wheelOptimization(x, y)[1];
        double turnAmountW = wheelOptimization(x, y)[0];
        this.robotTurnAmount = robotTurnAmount; //how much the robot heading should turn

        targetW = targetWOrientation;
        optimizedTargetW = clamp(currentW + turnAmountW);
        targetR = clamp(currentR + robotTurnAmount); //the target orientation is on a circle of (-179, 180]

        //setting PIDs for rotation of wheels & robot
        snapWheelPID.setTargets(optimizedTargetW, 0.01, 0, 0);
        tableSpinWheelPID.setTargets(targetR, 0.01, 0, 0);

        //specifically for auto (not teleop)
        splinemath.setInits(posSystem.getMotorClicks()[2], posSystem.getMotorClicks()[0]);
        splinemath.setPos(x, y, robotTurnAmount);
        linearmath.setInits(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
        linearmath.setPos(x, y, robotTurnAmount);
    }


    public void setSpin(){
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
            default:
                break;
        }
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

    public double[] getVelocity(){
        double[] motorPower = new double[4];

        motorPower[0] = constants.MAX_VELOCITY_DT * speed * (spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top left
        motorPower[1] = constants.MAX_VELOCITY_DT * speed * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom left
        motorPower[2] = constants.MAX_VELOCITY_DT * speed * (spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top right
        motorPower[3] = constants.MAX_VELOCITY_DT * speed * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rightThrottle * rotationPowerPercentage * rotationSwitchMotors); //bottom right

        return motorPower;
    }

}
