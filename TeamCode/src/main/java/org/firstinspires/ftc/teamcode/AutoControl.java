package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Camera.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.OpModes.Auto;
import org.firstinspires.ftc.teamcode.Susbsystem.Drive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.BooleanSupplier;

@Autonomous
public class AutoControl extends OpMode{
    OpticalSensor opticalSensor;
    public RobotClass robot;
    AprilTagPipeline aprilTagPipeline;

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
        telemetry.addData("motor 0 pos", robot.drivetrain.driveMotors[0].getCurrentPosition());
        telemetry.addData("motor 1 pos", robot.drivetrain.driveMotors[1].getCurrentPosition());
        telemetry.addData("motor 2 pos", robot.drivetrain.driveMotors[2].getCurrentPosition());
        telemetry.addData("motor 3 pos", robot.drivetrain.driveMotors[3].getCurrentPosition());
        telemetry.update();
        //opticalSensor = new OpticalSensor(drive);

    }

    @Override
    public void loop(){
        //aprilTagPipeline.updateAprilTagPipeline();
        telemetry.addData("motor 0 pos", robot.drivetrain.driveMotors[0].getCurrentPosition());
        telemetry.addData("motor 1 pos", robot.drivetrain.driveMotors[1].getCurrentPosition());
        telemetry.addData("motor 2 pos", robot.drivetrain.driveMotors[2].getCurrentPosition());
        telemetry.addData("motor 3 pos", robot.drivetrain.driveMotors[3].getCurrentPosition());
        telemetry.update();

        AutoDrive(10,0);
    }
    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 /  WHEEL_CIRCUMFERENCE_INCH;
    private final double kD = .01;
    @SuppressLint("DefaultLocale")
    public void AutoDrive(double targetDistance_INCH, double angle){

        angle = Math.toRadians(angle);

        angle -= robot.getHeading();

        double maxErrorAllowed = 1.1 * TICKS_PER_INCH;
        double kP = 0.5;
        double kI = 0.01;

        double integralSum = 0;
        double error, correction;
        double targetDistance = targetDistance_INCH * TICKS_PER_INCH;
        double[] wheelSpeeds = new double[4];

        while(!isStopRequested.getAsBoolean()){
            //SparkFunOTOS.Pose2D Pose2D = opticalSensor.getPos();
            telemetry.addData("targetDistance", targetDistance);
            telemetry.addData("maxErrorAllowed", maxErrorAllowed);
            telemetry.addLine(String.format("MotorPos %d %d %d %d",
                    robot.drivetrain.driveMotors[RobotClass.kFrontLeft].getCurrentPosition(),
                    robot.drivetrain.driveMotors[RobotClass.kFrontRight].getCurrentPosition(),
                    robot.drivetrain.driveMotors[RobotClass.kBackLeft].getCurrentPosition(),
                    robot.drivetrain.driveMotors[RobotClass.kBackRight].getCurrentPosition()));
            telemetry.update();
            error = targetDistance - robot.drivetrain.driveMotors[RobotClass.kFrontLeft].getCurrentPosition();
            //(targetDistance - Math.sqrt(Math.pow((targetDistance * Math.cos(angle)
            //- Pose2D.x), 2) + (Math.pow((targetDistance * Math.sin(angle)) - Pose2D.y, 2))));

            //correction = kP * error + kI * integralSum + kD * getDerivative(error, targetDistance);

            if(Math.abs(error) < maxErrorAllowed) {
                telemetry.addLine("ended");
                telemetry.update();
                stop = true;
                stop();
                break;
            }

            double x = Math.cos(angle);
            double y = -Math.sin(angle);

            double powerGroupY = (x * Math.cos(angle) - y * Math.sin(angle));
            double powerGroupX = (x * Math.sin(angle) - y * Math.cos(angle));
            double rotY;
            double rotX;
            if(error > 0){
                rotY = -powerGroupY;
                rotX = -powerGroupX;
            }
            else{
                rotY = powerGroupY;
                rotX = powerGroupX;
            }
            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY), 1.0);

            wheelSpeeds[RobotClass.kFrontLeft] = (rotY + rotX) / 3 / denominator;
            wheelSpeeds[RobotClass.kBackLeft] = (rotY - rotX) / 3/ denominator;
            wheelSpeeds[RobotClass.kFrontRight] = (rotY - rotX) / 3 / denominator;
            wheelSpeeds[RobotClass.kBackRight] = (rotY + rotX) / 3 / denominator;

            telemetry.addData("rotY", rotY);
            telemetry.addData("rotX", rotX);
            telemetry.addData("denominator", denominator);
            telemetry.addData("correction", 0);// correction);
            telemetry.addData("error", error);
            telemetry.addLine(String.format("wheelSpeeds %6.1f %6.1f %6.1f %6.1f", wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]));

            //Drive.normalizeRanges(wheelSpeeds);

            setPower(wheelSpeeds);
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
        error = targetDistance - robot.drivetrain.driveMotors[0].getCurrentPosition();
        return(kD * error ) / (previousError + 0.01);
    }
    @Override
    public void stop(){
        boolean stop = true;
        if(robot == null) return; // ensures that stop() is not called before initialization
        robot.drivetrain.driveMotors[0].setPower(0);
        robot.drivetrain.driveMotors[1].setPower(0);
        robot.drivetrain.driveMotors[2].setPower(0);
        robot.drivetrain.driveMotors[3].setPower(0);
    }
    private void setPower(double[] wheelSpeeds){
        robot.drivetrain.driveMotors[RobotClass.kFrontLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
        robot.drivetrain.driveMotors[RobotClass.kFrontRight].setPower(wheelSpeeds[RobotClass.kFrontRight]);
        robot.drivetrain.driveMotors[RobotClass.kBackLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
        robot.drivetrain.driveMotors[RobotClass.kBackRight].setPower(wheelSpeeds[RobotClass.kBackRight]);
    }

}