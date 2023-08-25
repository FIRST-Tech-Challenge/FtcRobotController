package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelDiffSwerveClass {
    // CONSTANTS
    static final double TICKS_PER_HALF_WHEEL_TURN = 433.5; // turn both motors this amount for 180 deg turn, was 458.62
    static final double TICKS_PER_INCH = 24.2; // Wheel movement. Pod motors turn opposite same amount
    static final double TICKS_PER_RAD_ROBOT_TURN = 47.7*TICKS_PER_INCH/(2.0*Math.PI); // Wheels must be perpendicular
    public double currentRobotAngle; // the angle of the robot
    public int targetPosition; // robot translation, in ticks

    // initial positions for each motor, in ticks, to get initial angle
    private int initialTurnTicks1;
    private int initialTurnTicks2;

    // HARDWARE
    // DcMotors
    DcMotor motor1a = null;
    DcMotor motor1b = null;
    DcMotor motor2c = null;
    DcMotor motor2d = null;

    // Constructor
    public TwoWheelDiffSwerveClass() {
        currentRobotAngle = 0;
        targetPosition = 0;
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

    public void initWheelAngles(double wheelAngle1, double wheelAngle2){

        initialTurnTicks1 =  (int)((wheelAngle1 / Math.PI) * TICKS_PER_HALF_WHEEL_TURN);

        initialTurnTicks2 =  (int)((wheelAngle2/ Math.PI) * TICKS_PER_HALF_WHEEL_TURN);

        setRunToPos(0.8);  // full speed  = 1.0. Backed off to prevent damage while developing
    }
    /**
     * setRunToPos executes the sequence required for using RUN_TO_POSITION
     */
    public void setRunToPos(double mtrSpeed) {
        motor1a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2c.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1a.setPower(mtrSpeed);
        motor1b.setPower(mtrSpeed);
        motor2c.setPower(mtrSpeed);
        motor2d.setPower(mtrSpeed);
        setMotorPositions(0,0,0);
        motor1a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRobotTranslation(int deltaRobotPosTicks) {
        targetPosition += deltaRobotPosTicks;
    }
    public void setMotorPositions(int turnPod1,int turnPod2,int robotHeadingTicks) {
        int ticksA,ticksB, ticksC,ticksD;

        // Add together initial pod angles, position ticks, pot turn ticks and robot turn ticks
        ticksA = initialTurnTicks2 + targetPosition + turnPod1 - robotHeadingTicks;
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
    public double getShortestTurnAngle(double currentWheelAng, double newAngle) {
        // newAngle will be from -PI to PI, because it uses ATAN2
        double deltaAngle;
        double currentRemainderAngle;
        double newRemainder;

        // Convert current angle into number from zero to 2*PI
        currentRemainderAngle = moduloAngle(currentWheelAng);

        // Convert new angle into number from -PI to PI
        newRemainder = moduloAngle(newAngle);
        if(newRemainder>Math.PI) newRemainder -= (2.0*Math.PI);

        // Find the shortest turn to newAngle
        deltaAngle = newRemainder - currentRemainderAngle;
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
