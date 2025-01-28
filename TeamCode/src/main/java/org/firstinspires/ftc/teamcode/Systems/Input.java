package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Input {

    Motors motors;
    Servos servos;
    IMU imu;
    ElapsedTime elapsedTime;

    private int setPoint;
    private double integral = 0;
    private double prevError = 0;  // Previous error, used for derivative
    private double prevTime;

    // Robot position
    private double x = 0.0; // X position (strafe) in inches
    private double y = 0.0; // Y position (forward/backward) in inches
    private double heading = 0.0; // Robot's current heading in radians


    private boolean isMove = false;
    private boolean isStrafe = false;
    private boolean isSpin = false;

    //    FtcDashboard dashboard;
//    Telemetry dashboardTelemetry;

    public Input(HardwareMap hardwareMap, Boolean isAutonomous) {
        motors = new Motors(hardwareMap, isAutonomous);
        servos = new Servos(hardwareMap);
        imu = new IMU(hardwareMap);

        elapsedTime = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        setPoint = motors.getArmRestingPosition();
        prevTime = elapsedTime.milliseconds();
    }

    public void move(double power) {

            if (!isSpin) {
                motors.MoveMotor(Motors.Type.LeftBack, -power);
                motors.MoveMotor(Motors.Type.LeftFront, -power);
                motors.MoveMotor(Motors.Type.RightFront, -power);
                motors.MoveMotor(Motors.Type.RightBack, -power);
                //prob fix movement forward
            }

            if (power != 0) {
                isMove = true;
            }
            else {
                isMove = false;
            }
    }

    public void strafe(double power) {

            motors.MoveMotor(Motors.Type.LeftFront, power); // left front
            motors.MoveMotor(Motors.Type.RightFront, -power); // right front

            motors.MoveMotor(Motors.Type.LeftBack, -power); // left back
            motors.MoveMotor(Motors.Type.RightBack, power); // right back

            if (power != 0) {
                isStrafe = true;
            }
            else {
                isStrafe = false;
            }
    }

    public void spin(double power) {

            if(!isMove) {
                motors.MoveMotor(Motors.Type.LeftFront, power); // left front
                motors.MoveMotor(Motors.Type.LeftBack, power); // left back

                motors.MoveMotor(Motors.Type.RightFront, -power); // right front
                motors.MoveMotor(Motors.Type.RightBack, -power); // right back
            }

            if (power != 0) {
                isSpin = true;
            }
            else {
                isSpin = false;
            }
    }

    public boolean isMove() {
        return isMove;
    }

    public boolean isStrafe() {
        return isStrafe;
    }

    public boolean isSpin() {
        return isSpin;
    }

    public void claw(boolean openClaw, boolean closeClaw) {

        if (closeClaw) {
            servos.moveServo(Servos.Type.Claw, Constants.SERVO_OPEN);
        }
        else if (openClaw) {
            servos.moveServo(Servos.Type.Claw, Constants.SERVO_CLOSED);
        }
    }

    public void upArm(double power) {


            //double thing = (-(Math.abs(motors.getArmPosition() - 440)) / 5.6) + 100;
//up 1435 to 1030
            double realPower = 0;
            if (((motors.getArmPosition() < 1650) && (motors.getArmPosition() > 800)) || (motors.getArmPosition() > 2250) || (power > 0))  {
                realPower = power;
                if (motors.getArmPosition() > 2250) {
                    if ((motors.getUpArmPosition() <= -2400) && (power > 0)) {
                        realPower = 0;
                    }
                }
            }
            else {
                realPower = 0;
            }

            motors.MoveMotor(Motors.Type.UpArm, realPower);

            BotTelemetry.addData("real Power" , realPower);
            BotTelemetry.addData("upArm Pos", motors.getUpArmPosition());
    }



    public void arm(int targetPos) {

        setPoint = targetPos;

        // Clamp setPoint between resting and reaching positions
        setPoint = Math.max(motors.getArmRestingPosition(), Math.min(setPoint, motors.getArmReachingPosition()));

        // Get current time and arm position
        double time = elapsedTime.milliseconds();

        // Time difference (dt)
        double dt = (time - prevTime) / 1000.0;  // Convert to seconds

        double processValue = motors.getArmPosition(); // Finding the position of the motors

        // Calculate error
        double errorValue = setPoint - processValue;

        // Prevent divide-by-zero errors
        if (dt == 0) {
            dt = 0.1;  // Default small value if no time has passed
        }

        // Calculate PID terms
        double proportional = Constants.KP * errorValue;

        integral += errorValue * dt;  // Integrate the error over time
        // Anti-windup: Limit the integral term to prevent it from growing too large
        integral = Math.max(Math.min(integral, 1000), -1000);

        double derivative = (errorValue - prevError) / dt;

        // Compute the final PID output
        double output = proportional + Constants.KI * integral + Constants.KD * derivative;

        // Apply the motor power
        output = Math.max(Math.min(output, 100), -100);  // Clamp output to motor range and make it so that it will more slowly go to its target position

        if((motors.getArmPosition() == motors.getArmRestingPosition()) && (targetPos == 0)) { // Allows the arm to not be powered when it is in its resting position and no inputs are given
            motors.MoveMotor(Motors.Type.Arm,0);                                // This can prevent the motor from overheating like it was doing earlier
        }
        else {
            motors.MoveMotor(Motors.Type.Arm, output);
        }



        // Store current error and time for next iteration
        prevError = errorValue;
        prevTime = time;


        BotTelemetry.addData("Set Point", setPoint);
        BotTelemetry.addData("Process Value", processValue);
        BotTelemetry.addData("Proportional Gain", Constants.KP);
        BotTelemetry.addData("Integral Gain", Constants.KI);
        BotTelemetry.addData("Derivative Gain", Constants.KD);
        BotTelemetry.addData("Proportional", proportional);
        BotTelemetry.addData("Integral", integral);
        BotTelemetry.addData("Derivative", derivative);
        BotTelemetry.addData("PID Output", output);


    }

    public double getTravelledDistance() {
        double leftBackPosition = motors.getLeftFrontPosition();
        double rightBackPosition = motors.getRightFrontPosition();

        // Get the current position of the motor

        double lbrevolutions = leftBackPosition / Constants.COUNT_PER_REVOLUTION;
        double lbangle = lbrevolutions * 360;
        //double lbangleNormalized = lbangle % 360;

        double rbrevolutions = rightBackPosition / Constants.COUNT_PER_REVOLUTION;
        double rbangle = rbrevolutions * 360;
        //double rbangleNormalized = rbangle % 360;


        double rbdistance = Constants.WHEEL_CIRCUMFERENCE * rbrevolutions * Constants.FRICTION_PERCENT;
        double lbdistance = Constants.WHEEL_CIRCUMFERENCE * lbrevolutions * Constants.FRICTION_PERCENT;

        double distance;

        //reset motors after calculations;
        if(!isSpin()) {
            return distance = (Math.floor(((lbdistance + rbdistance))/2))  / 25.4;
        }
        else {
            return distance = ((lbdistance - rbdistance)/2)  / 25.4;
        }
    }

    public void resetDistance() {
        motors.resetDistance();
    }

    public int getArmPos() {
        return motors.getArmPosition();
    }

    public int getUpArmPos () {
        return  motors.getUpArmPosition();
    }


}
