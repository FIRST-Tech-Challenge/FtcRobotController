package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Susbsystem.Drive;

import java.util.function.BooleanSupplier;

@Autonomous
public class AutoControl extends OpMode{
    OpticalSensor opticalSensor;
    Drive drive;
    boolean stop = false;
    BooleanSupplier isStopRequested = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return stop;
        }
    };
    @Override
    public void init(){
        drive = new Drive(hardwareMap);
        opticalSensor = new OpticalSensor(drive);

    }
    @Override
    public void start(){

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

        telemetry.addData("motor 0 pos", drive.drivetrain.driveMotors[0].getCurrentPosition());
        telemetry.update();

        angle -= drive.getHeading();

        double maxErrorAllowed = 1.1 * TICKS_PER_INCH;
        double kP = 0.5;
        double kI = 0.01;

        double integralSum = 0;
        double error, correction;
        double targetDistance = targetDistance_INCH * TICKS_PER_INCH;

        drive.drivetrain.driveMotors[0].setDirection(DcMotorSimple.Direction.FORWARD); //FLM
        drive.drivetrain.driveMotors[1].setDirection(DcMotorSimple.Direction.FORWARD); //BLM
        drive.drivetrain.driveMotors[2].setDirection(DcMotorSimple.Direction.REVERSE); //BRM
        drive.drivetrain.driveMotors[3].setDirection(DcMotorSimple.Direction.FORWARD);

        while(!isStopRequested.getAsBoolean()){

            //SparkFunOTOS.Pose2D Pose2D = opticalSensor.getPos();

            error = targetDistance - drive.drivetrain.driveMotors[0].getCurrentPosition();
            //(targetDistance - Math.sqrt(Math.pow((targetDistance * Math.cos(angle)
            //- Pose2D.x), 2) + (Math.pow((targetDistance * Math.sin(angle)) - Pose2D.y, 2))));

            correction = kP * error + kI * integralSum + kD * getDerivative(error, targetDistance);

            if(Math.abs(error) <  maxErrorAllowed) {
                stop = true;
                stop();
                break;
            }

            double x = error * Math.cos(angle);
            double y = error * Math.sin(angle);

            double rotY = x * Math.cos(-angle) - y * Math.sin(-angle);
            double rotX = x * Math.sin(angle) - y * Math.cos(angle);

            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY), 1.0);




            drive.drivetrain.driveMotors[RobotClass.kFrontLeft].setPower((((rotY + rotX) / denominator)) / correction);
            drive.drivetrain.driveMotors[RobotClass.kBackLeft].setPower((((rotY - rotX) / denominator)) / correction);
            drive.drivetrain.driveMotors[RobotClass.kFrontRight].setPower((((rotY - rotX) / denominator)) / correction);
            drive.drivetrain.driveMotors[RobotClass.kBackRight].setPower((((rotY + rotX) / denominator)) / correction);

        }
        drive.drivetrain.setPowerBasic(0.0);
    }
    public void AutoTurn(double angle){
        double loopCounter = 0;
        boolean atTarget = false;
        double angularDistance = 0;

        double initialAngle = drive.getHeading();

        do{
            double turnVal = 1;


            if(angle - initialAngle < 0) turnVal = -1; // checks which way it should turn

            double currentAngle = drive.getHeading();

            angularDistance = Math.abs(currentAngle - angle);
            if(angularDistance > 180){ // dealing with edge case
                turnVal *= -1;
                angularDistance = 360 - angularDistance;
            }
            double powerReduce = angularDistance / 90;

            powerReduce = Math.max(powerReduce, 0.2);
            powerReduce = Math.min(powerReduce, 1);

            drive.drivetrain.driveMotors[0].setPower(turnVal * powerReduce);
            drive.drivetrain.driveMotors[1].setPower(turnVal * powerReduce);
            drive.drivetrain.driveMotors[2].setPower(-turnVal * powerReduce);
            drive.drivetrain.driveMotors[3].setPower(-turnVal * powerReduce);

            if(angularDistance < 1.5) atTarget = true;
        }
        while(!atTarget);
    }

    private double getDerivative(double error, double targetDistance){
        double previousError = error;
        error = targetDistance - drive.drivetrain.driveMotors[0].getCurrentPosition();
        return(kD * error ) / (previousError + 0.01);
    }
    @Override
    public void stop(){
        boolean stop = true;
        if(drive == null) return; // ensures that stop() is not called before initialization
        drive.drivetrain.driveMotors[0].setPower(0);
        drive.drivetrain.driveMotors[1].setPower(0);
        drive.drivetrain.driveMotors[2].setPower(0);
        drive.drivetrain.driveMotors[3].setPower(0);
    }
}