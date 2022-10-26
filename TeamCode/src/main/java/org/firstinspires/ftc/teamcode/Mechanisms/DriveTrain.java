package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {
    Telemetry telemetry;
    public DcMotor FrontLeft;
    public DcMotor BackLeft;
    public DcMotor FrontRight;
    public DcMotor BackRight;

    double PULSES_PER_ROTATION = 537.5;
    double GEAR_RATIO = 1;
    double WHEEL_DIAMETER_INCHES = 3.77953;
    double TRACK_WIDTH = 14;
    double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    //double COUNTS_PER_INCH = PULSES_PER_ROTATION / (WHEEL_DIAMETER_INCHES * Math.PI);


    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        setup(hardwareMap);
    }
    public void setup(HardwareMap hardwareMap) {
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackRight = hardwareMap.dcMotor.get("BackRight");

        setMotorDirection(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE );
    }

    public int convertDistTicks( double distanceToTravel, double circumference ) {
        double revolutions = distanceToTravel / circumference;
        int totalTicks = (int) Math.round( (revolutions * PULSES_PER_ROTATION) / GEAR_RATIO );

        return totalTicks;
    }

    public int convertTicksDist( double ticksToTravel, double circumference ) {
        double calculations = ticksToTravel * circumference * GEAR_RATIO;
        int totalDistance = (int) Math.round( calculations / PULSES_PER_ROTATION );

        return totalDistance;
    }

    public void setMotorDirection(DcMotorSimple.Direction flDirection, DcMotorSimple.Direction frDirection, DcMotorSimple.Direction blDirection, DcMotorSimple.Direction brDirection){
        FrontLeft.setDirection(flDirection);
        BackLeft.setDirection(blDirection);
        FrontRight.setDirection(frDirection);
        BackRight.setDirection(brDirection);
    }

    public void setMotorPower( double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower ) {
        FrontLeft.setPower( frontLeftPower );
        BackLeft.setPower( backLeftPower );
        FrontRight.setPower( frontRightPower );
        BackRight.setPower( backRightPower );
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior flBehavior, DcMotor.ZeroPowerBehavior blBehavior, DcMotor.ZeroPowerBehavior frBehavior, DcMotor.ZeroPowerBehavior brBehavior){
        FrontLeft.setZeroPowerBehavior( flBehavior );
        BackLeft.setZeroPowerBehavior( blBehavior );
        FrontRight.setZeroPowerBehavior( frBehavior );
        BackRight.setZeroPowerBehavior( brBehavior );
    }
    public void setMotorEncoderPosition(int flPosition,int blPosition, int frPosition,int brPosition){
        FrontLeft.setTargetPosition(flPosition);
        FrontRight.setTargetPosition(frPosition);
        BackLeft.setTargetPosition(blPosition);
        BackRight.setTargetPosition(brPosition);
    }
    public void setAllPower( double allPower) {
        FrontLeft.setPower(allPower);
        BackLeft.setPower(allPower);
        FrontRight.setPower(allPower);
        BackRight.setPower(allPower);
    }
    public void setMotorModeRunToPosition()
    {
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotorModeStopAndResetEncoder()
    {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMotorModeRunUsingEncoder()
    {
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveRobotToPosition(double power, double distanceInches) {
        // Determine new target position, and pass to motor controller
        int newTargetPosition = convertDistTicks(distanceInches,CIRCUMFERENCE);
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-newTargetPosition, -newTargetPosition, -newTargetPosition, -newTargetPosition);
        // Turn On RUN_TO_POSITION
        setMotorModeRunToPosition();
        //start motion
        setMotorPower(power, power, power, power);
        // keep looping while we are still active, and there is time left, and all motors are running.
        while (BackLeft.isBusy() && FrontLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()) {
        }
        // Stop all motion;
        setAllPower(0.0);
        // Turn off RUN_TO_POSITION
        setMotorModeRunUsingEncoder();
    }

    public void MoveRobotToPositionStrafe(double power, double distanceInches){
        int newTargetPosition = convertDistTicks(distanceInches,CIRCUMFERENCE);
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-newTargetPosition, newTargetPosition, newTargetPosition, -newTargetPosition);
        setMotorModeRunToPosition();
        setMotorPower(power,-power,power,-power);
        while (FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy() ) {
        }
    }
    public void pointTurn(double angle, double power){
        // Determine new target position, and pass to motor controller
        double dist = (TRACK_WIDTH*Math.PI)*(angle/360);
        int ticks = (int) ((dist/(Math.PI*WHEEL_DIAMETER_INCHES))*PULSES_PER_ROTATION);
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-ticks,-ticks,ticks,ticks);
        // Turn on RUN_TO_POSITION
        setMotorModeRunToPosition();
        //start the motion
        setMotorPower(power,power,power,power);
        // keep looping while we are still active, and there is time left, and all motors are running.
        while (BackLeft.isBusy() && FrontLeft.isBusy() && BackRight.isBusy() && FrontRight.isBusy() ){
        }
        // Stop all motion;
        setAllPower(0.0);
        // Turn off RUN_TO_POSITION
        setMotorModeRunUsingEncoder();
    }
}
