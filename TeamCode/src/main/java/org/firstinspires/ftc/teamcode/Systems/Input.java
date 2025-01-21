package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public Input(HardwareMap hardwareMap) {
        motors = new Motors(hardwareMap);
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

    public void claw(boolean grabButton, boolean releaseButton) {

        if (releaseButton) {
            servos.moveServo(Servos.Type.Claw, Constants.SERVO_OPEN);
        }
        else if (grabButton) {
            servos.moveServo(Servos.Type.Claw, Constants.SERVO_CLOSED);
        }
    }

    public void upArm(double power) {


            //double thing = (-(Math.abs(motors.getArmPosition() - 440)) / 5.6) + 100;
//up 1435 to 1030
            double realPower = 0;
            if (((motors.getArmPosition() < 1400) && (motors.getArmPosition() > 800)) || (motors.getArmPosition() > 2250) || (power > 0))  {
                realPower = power;
            }
            else {
                realPower = 0;
            }

            motors.MoveMotor(Motors.Type.UpArm, realPower);

            BotTelemetry.addData("real Power" , realPower);
            BotTelemetry.addData("upArm Pos", motors.getUpArmPosition());
    }



    public void arm(double gamepad) {

        setPoint += (int) (-gamepad * 35);    // Multiply the game pad input by a number so that we can tune the sensitivity then turn it into and int so the code can work

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

        if((motors.getArmPosition() == motors.getArmRestingPosition()) && (gamepad == 0)) { // Allows the arm to not be powered when it is in its resting position and no inputs are given
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


    public void calculatePosition() {
        heading = imu.getAngle('y');

        // Read encoder values
        double flTicks = motors.getLeftFrontPosition();
        double frTicks = motors.getLeftBackPosition();
        double blTicks = motors.getRightFrontPosition();
        double brTicks = motors.getRightBackPosition();

        // Convert encoder ticks to distances
        double flDistance = (flTicks / Constants.TICKS_PER_REVOLUTION) * Constants.WHEEL_CIRCUMFERENCE;
        double frDistance = (frTicks / Constants.TICKS_PER_REVOLUTION) * Constants.WHEEL_CIRCUMFERENCE;
        double blDistance = (blTicks / Constants.TICKS_PER_REVOLUTION) * Constants.WHEEL_CIRCUMFERENCE;
        double brDistance = (brTicks / Constants.TICKS_PER_REVOLUTION) * Constants.WHEEL_CIRCUMFERENCE;

        // Calculate movement in robot-centric coordinates
        double forward = (flDistance + frDistance + blDistance + brDistance) / 4.0;
        double strafe = (-flDistance + frDistance + blDistance - brDistance) / 4.0;

        // Transform robot-centric motion to field-centric motion
        double deltaX = forward * Math.sin(heading) + strafe * Math.cos(heading);
        double deltaY = forward * Math.cos(heading) - strafe * Math.sin(heading);

        // Update robot's position
        x += deltaX;
        y += deltaY;

        BotTelemetry.drawPosition(x, y, heading);
        // Pause briefly
        //sleep(50); // 50ms delay for smoother updates
    }
}
