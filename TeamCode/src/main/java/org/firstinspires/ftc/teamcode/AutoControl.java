package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Camera.AprilTagPipeline;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

@Autonomous
public class AutoControl extends OpMode{
    OpticalSensor opticalSensor;
    public RobotClass robot;
    AprilTagPipeline aprilTagPipeline;
    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 /  WHEEL_CIRCUMFERENCE_INCH;

    Map<RobotClass.MOTORS, Double> wheelSpeeds = new HashMap<>();

    boolean stop = false;
    BooleanSupplier isStopRequested = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return stop;
        }
    };
    @Override
    public void init(){
        robot = new RobotClass(hardwareMap);
        robot.stopAndReset();
        robot.setDirection();
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //aprilTagPipeline = new AprilTagPipeline(robot.webcamName, telemetry);
        //opticalSensor = new OpticalSensor(drive);
    }

    @Override
    public void start(){
        AutoTurn(180);
        //AutoDrive(6, 45);
        //simpleDriveForward();
    }

    @Override
    public void loop(){
        //aprilTagPipeline.updateAprilTagPipeline();
    }

    private final double kD = .01;
    @SuppressLint("DefaultLocale")
    public void AutoDrive(double targetDistance_INCH, double angle){
        angle = Math.toRadians(angle);
        //angle -= robot.getHeading();

        double maxErrorAllowed = 1.1 * TICKS_PER_INCH;
        double kP = 0.5;
        double kI = 0.01;
        double integralSum = 0;
        double error, correction;
        double targetDistance = targetDistance_INCH * TICKS_PER_INCH;

        while(!isStopRequested.getAsBoolean()){
            //SparkFunOTOS.Pose2D Pose2D = opticalSensor.getPos();

            error = targetDistance - (robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrentPosition());
            //(targetDistance - Math.sqrt(Math.pow((targetDistance * Math.cos(angle)
            //- Pose2D.x), 2) + (Math.pow((targetDistance * Math.sin(angle)) - Pose2D.y, 2))));

            //correction = kP * error + kI * integralSum + kD * getDerivative(error, targetDistance);
            if(Math.abs(error) < maxErrorAllowed) {
                stop = true;
                stop();
                break;
            }
            //@ 45 x = 0.707106
            //@ 45 y = 0.707106
            //@ 90 x = 0
            //@ 90 y = 1
            double x = Math.cos(angle);
            double y = Math.sin(angle);

            //double powerGroupX = (Math.cos(-angle) - Math.sin(-angle));
            //double powerGroupY = (Math.sin(-angle) + Math.cos(-angle));

            double rotY = y;
            double rotX = x;
//            if(error < 0){
//                rotY = -powerGroupY;
//                rotX = -powerGroupX;
//            }
//            else{
//                rotY = powerGroupY;
//                rotX = powerGroupX;
//            }
            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY), 1.0);

            wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, (rotY + rotX) / 2.0 / denominator );
            wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, (rotY - rotX) / 2.0 / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, (rotY - rotX) / 2.0 / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, (rotY + rotX) / 2.0 / denominator);


            telemetry.addData("rotY", rotY);
            telemetry.addData("rotX", rotX);
            telemetry.addData("denominator", denominator);
            telemetry.addData("correction", 0);// correction);
            telemetry.addData("error", error);
            telemetry.addLine(String.format("wheelSpeeds %6.4f %6.4f %6.4f %6.4f", wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT), wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT), wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT), wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT)));
            telemetry.addData("targetDistance", targetDistance);
            telemetry.addData("maxErrorAllowed", maxErrorAllowed);
            telemetry.addData("MotorPos", robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrentPosition());

            telemetry.update();

            //Drive.normalizeRanges(wheelSpeeds);

            UpdateWheelPowers();
        }

    }

    public void AutoTurn(double angle){
        double loopCounter = 0;
        boolean atTarget = false;
        double angularDistance = 0;

        double initialAngle = robot.getHeading();

        do{
            double turnVal = 1;
            if(angle - initialAngle < 0) turnVal = -1; // checks which way it should turn

            double currentAngle = Math.toDegrees(robot.getHeading());

            angularDistance = Math.abs(currentAngle - angle);
            if(angularDistance > 180){ // dealing with edge case
                turnVal *= -1;
                angularDistance = 360 - angularDistance;
            }

            double powerReduce = angularDistance / (angle / 1.5);

            powerReduce = Math.max(powerReduce, 0.15);
            powerReduce = Math.min(powerReduce, 0.9);

            wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, -turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, -turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, turnVal * powerReduce);

            telemetry.addData("angularDistance", angularDistance);
            telemetry.addData("turnVal", turnVal);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("powerReduce", powerReduce);
            telemetry.update();

            UpdateWheelPowers();

            if(angularDistance < 1.5) atTarget = true;
        }
        while(!atTarget);
        stop();
    }
    public void simpleDriveForward(){
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, 0.3);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, 0.3);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, 0.3);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, 0.3);
        UpdateWheelPowers();
//        try {
//            Thread.sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        stop();
    }

    public void stopMotors(){
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, 0.0);
        UpdateWheelPowers();
//        try {
//            Thread.sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        stop();
    }

    private double getDerivative(double error, double targetDistance){
        double previousError = error;
        error = targetDistance - robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrentPosition();
        return(kD * error ) / (previousError + 0.01);
    }
    @Override
    public void stop(){
        boolean stop = true;
        if(robot == null) return; // ensures that stop() is not called before initialization
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0.0);
        requestOpModeStop();
    }
    private void UpdateWheelPowers(){
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT));
    }

}