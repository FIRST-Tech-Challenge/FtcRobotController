package org.firstinspires.ftc.teamcode.AutoControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;

public class AutoControl extends OpMode{
    RobotClass robot;
    @Override
    public void init(){
        robot = new RobotClass(hardwareMap);
    }

    @Override
    public void loop(){

    }

    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 /  WHEEL_CIRCUMFERENCE_INCH;
    private final double kD = .01;
    public void AutoDrive(double targetDistance_INCH, double angle){

        angle -= robot.getHeading();

        double maxErrorAllowed = 1.1 * TICKS_PER_INCH;
        double kP = 0.5;
        double kI = 0.01;

        double integralSum = 0;
        double error, correction;
        double targetDistance = targetDistance_INCH * TICKS_PER_INCH;

        while(true){

            error = targetDistance - robot.drivetrain.driveMotors[0].getCurrentPosition();
            correction = kP * error + kI * integralSum + kD * getDerivative(error, targetDistance);

            if(Math.abs(error) <  maxErrorAllowed) {
                break;
            }

            double x = error * Math.cos(angle);
            double y = error * Math.sin(angle);

            double rotY = x * Math.cos(-angle) - y * Math.sin(-angle);
            double rotX = x * Math.sin(angle) - y * Math.cos(angle);

            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY), 1.0);

            robot.drivetrain.driveMotors[0].setPower((((rotY + rotX) / denominator)) / correction);
            robot.drivetrain.driveMotors[1].setPower((((rotY - rotX) / denominator)) / correction);
            robot.drivetrain.driveMotors[2].setPower((((rotY - rotX) / denominator)) / correction);
            robot.drivetrain.driveMotors[3].setPower((((rotY + rotX) / denominator)) / correction);

        }
        robot.drivetrain.setPowerBasic(0.0);
    }
    public void AutoTurn(double angle){
        double loopCounter = 0;
        boolean atTarget = false;
        double angularDistance = 0;

        double initialAngle = robot.getHeading();

        do{
            double turnVal = 1;


            if(angle - initialAngle < 0) turnVal = -1; // checks which way it should turn

            double currentAngle = robot.getHeading();

            angularDistance = Math.abs(currentAngle - angle);
            if(angularDistance > 180){ // dealing with edge case
                turnVal *= -1;
                angularDistance = 360 - angularDistance;
            }
            double powerReduce = angularDistance / 90;

            powerReduce = Math.max(powerReduce, 0.2);
            powerReduce = Math.min(powerReduce, 1);

            robot.drivetrain.driveMotors[0].setPower(turnVal * powerReduce);
            robot.drivetrain.driveMotors[1].setPower(turnVal * powerReduce);
            robot.drivetrain.driveMotors[2].setPower(-turnVal * powerReduce);
            robot.drivetrain.driveMotors[3].setPower(-turnVal * powerReduce);

            if(angularDistance < 1.5) atTarget = true;
        }
        while(!atTarget);
    }

    private double getDerivative(double error, double targetDistance){
        double previousError = error;
        error = targetDistance - robot.drivetrain.getAvgPos();
        return(kD * error ) / (previousError + 0.01);
    }

    @Override
    public void stop(){

        if(robot == null) return; // ensures that stop() is not called before initialization
        robot.drivetrain.driveMotors[0].setPower(0);
        robot.drivetrain.driveMotors[1].setPower(0);
        robot.drivetrain.driveMotors[2].setPower(0);
        robot.drivetrain.driveMotors[3].setPower(0);
    }
}