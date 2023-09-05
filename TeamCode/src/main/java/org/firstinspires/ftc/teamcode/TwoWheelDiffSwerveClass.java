package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Two Wheeled Differential Swerve Drive Class
 * Local system = Positive X is the line from wheel 2 to wheel 1
 */
public class TwoWheelDiffSwerveClass {
    // CONSTANTS
    static final double TICKS_PER_RAD_WHEEL_TURN = 137.987; // turn both motors 433.5 ticks for 180 deg turn
    static final double TICKS_PER_INCH = 24.2; // Wheel movement. Pod motors turn opposite same amount
    static final double WHEEL_DISTANCE = 15.2; // Distance between wheels, in inches.
    static final double TICKS_PER_RAD_ROBOT_TURN = WHEEL_DISTANCE*Math.PI*TICKS_PER_INCH/(2.0*Math.PI); // Wheels must be perpendicular
    public double robotAngle; // the angle of the robot, radians
    public double wheel1Angle; // the angle of wheel/pod 1, radians
    public double wheel2Angle; // the angle of wheel/pod 2, radians
    public int targetPosition; // robot cumulative translation at center, in ticks

    // initial positions for each wheel (motor pair), in ticks, to get initial wheel angle
    private int initialTurnTicks1 = 0;
    private int initialTurnTicks2 = 0;

    // Used to reverse wheel direction to minimize pod turning
    private int wheel1Sign;
    private int wheel2Sign;

    // HARDWARE
    // DcMotors
    DcMotor motor1a = null;
    DcMotor motor1b = null;
    DcMotor motor2c = null;
    DcMotor motor2d = null;

    // Constructor
    public TwoWheelDiffSwerveClass() {
        this.robotAngle = 0;
        this.wheel1Angle = 0;
        this.wheel2Angle = 0;
        this.targetPosition = 0;
        this.wheel1Sign = 1;
        this.wheel2Sign = 1;
    }
    public void initDrive(HardwareMap hwMap) {
        // Define and Initialize Motors
        motor1a = hwMap.get(DcMotor.class, "a");
        motor1b = hwMap.get(DcMotor.class, "b");
        motor2c = hwMap.get(DcMotor.class, "c");
        motor2d = hwMap.get(DcMotor.class, "d");

        motor1a.setDirection(DcMotor.Direction.FORWARD);
        motor1b.setDirection(DcMotor.Direction.REVERSE); // Why??
        motor2c.setDirection(DcMotor.Direction.FORWARD);
        motor2d.setDirection(DcMotor.Direction.FORWARD);
    }

    public void initWheelAngles(double isWheelAngle1, double isWheelAngle2, double tobeWheelAngle1, double tobeWheelAngle2) {
        double deltaAngle1, deltaAngle2;
        deltaAngle1 = getShortestTurnAngle(isWheelAngle1,tobeWheelAngle1,1);
        deltaAngle2 = getShortestTurnAngle(isWheelAngle2,tobeWheelAngle2,2);

        initialTurnTicks1 =  (int)(deltaAngle1 * TICKS_PER_RAD_WHEEL_TURN)*wheel1Sign;
        initialTurnTicks2 =  (int)(deltaAngle2 * TICKS_PER_RAD_WHEEL_TURN)*wheel2Sign;

        setRunToPos(0.5);  // full speed  = 1.0. Backed off to prevent damage while developing
    }
    public void setMotorsPower(double mtrSpeed) {
        motor1a.setPower(mtrSpeed);
        motor1b.setPower(mtrSpeed);
        motor2c.setPower(mtrSpeed);
        motor2d.setPower(mtrSpeed);
    }
    /**
     * setRunToPos executes the sequence required for using RUN_TO_POSITION
     */
    public void setRunToPos(double mtrSpeed) {
        motor1a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2c.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsPower(mtrSpeed);
        setMotorPositions(0,0,0);
        motor1a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRobotTranslation(int deltaRobotPosTicks) {
        targetPosition += deltaRobotPosTicks;
    }
    public void setMotorPositions(double newWheel1Angle, double newWheel2Angle,double deltaRobotAngle) {
        int ticksA,ticksB, ticksC,ticksD;
        int turnPod1, turnPod2;
        int robotHeadingTicks;

        robotAngle += deltaRobotAngle;
        robotHeadingTicks = (int) (robotAngle * TICKS_PER_RAD_ROBOT_TURN); // convert robot heading to ticks

        wheel1Angle = newWheel1Angle;
        //wheel1Angle = getShortestTurnAngle(wheel1Angle,newWheel1Angle,1);
        turnPod1 = (int) (wheel1Angle* TICKS_PER_RAD_WHEEL_TURN)*wheel1Sign;

        wheel2Angle = newWheel2Angle;
        //wheel2Angle = getShortestTurnAngle(wheel2Angle,newWheel2Angle,2);
        turnPod2 = (int) (wheel2Angle* TICKS_PER_RAD_WHEEL_TURN)*wheel2Sign;

        // Add together initial pod angles, position ticks, pot turn ticks and robot turn ticks
        ticksA = initialTurnTicks1 + targetPosition + turnPod1 - robotHeadingTicks;
        ticksB = initialTurnTicks1 - targetPosition + turnPod1 + robotHeadingTicks;
        ticksC = initialTurnTicks2 + targetPosition + turnPod2 + robotHeadingTicks;
        ticksD = initialTurnTicks2 - targetPosition + turnPod2 - robotHeadingTicks;

        motor1a.setTargetPosition(ticksA);
        motor1b.setTargetPosition(ticksB);
        motor2c.setTargetPosition(ticksC);
        motor2d.setTargetPosition(ticksD);
    }

    /**
     * Returns the shortest turn angle
     * @param currentWheelAng Current Wheel Angle
     * @param newAngle Desired wheel angle
     * @return The smallest delta angle to get to the new angle
     */
    public double getShortestTurnAngle(double currentWheelAng, double newAngle, int pod) {
        // newAngle will be from -PI to PI, because it uses ATAN2
        double deltaAngle;
        double currentRemainderAngle;
        double newRemainder;
        int sign;

        // Convert current angle into number from zero to 2*PI
        currentRemainderAngle = moduloAngle(currentWheelAng);

        // Convert new angle into number from -PI to PI
        newRemainder = moduloAngle(newAngle);
        if(newRemainder>Math.PI) newRemainder -= (2.0*Math.PI);

        // Find the shortest turn to newAngle
        deltaAngle = newRemainder - currentRemainderAngle;
        /*
        if(deltaAngle<=-1.5*Math.PI) {
            deltaAngle = currentRemainderAngle-(newRemainder+2.0*Math.PI);
            sign = 1;
        } else if(deltaAngle<=-0.5*Math.PI) {
            deltaAngle = newRemainder-(currentRemainderAngle-Math.PI);
            sign = -1;
        } else if(deltaAngle<=0.5*Math.PI) {
            deltaAngle = newRemainder - currentRemainderAngle;
            sign = 1;
        } else if(deltaAngle<=1.5*Math.PI) {
            deltaAngle = newRemainder-(currentRemainderAngle+Math.PI);
            sign = -1;
        } else {
            deltaAngle = newRemainder-(currentRemainderAngle+2.0*Math.PI);
            sign = 1;
        }

        if(pod==1) wheel1Sign = sign;
        else wheel2Sign = sign;
*/
        if(deltaAngle < -Math.PI) {
            deltaAngle = newAngle + (2.0*Math.PI) - currentRemainderAngle;
        }

        return deltaAngle;
    }
    /**
     * Returns an angle from 0 to 2*PI for any real angle
     * @param inAngle any real angle
     * @return an angle from 0 to 2*PI
     */
    public double moduloAngle(double inAngle) {
        double currentRemainderAngle;

        // Convert current angle into number from zero to 2*PI
        currentRemainderAngle = inAngle % (2.0*Math.PI);
        // Java modulo can return negative remainder, so the next step is needed
        if (currentRemainderAngle < 0.0) currentRemainderAngle += 2.0*Math.PI;
        return currentRemainderAngle;
    }


}
