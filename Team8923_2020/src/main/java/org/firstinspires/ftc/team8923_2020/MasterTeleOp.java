package org.firstinspires.ftc.team8923_2020;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;

import java.util.Timer;

abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;
    private boolean prevClawPosition = false;

    private Toggle driveSpeedToggle = new Toggle();
    private Toggle shooterOn = new Toggle();
    private Toggle aPressed = new Toggle();
    private Toggle powerShot = new Toggle();

    boolean isliftUp = false;
    boolean isShooterRunning = false;
    boolean isPowerShot = false;
    boolean isFlickerRunning = false;
    boolean startScoring = false;

    boolean justSwitchedWobbleModes = true;
    int flipWobbleState = 0;
    boolean wobbleFlipped = false;
    double driveSpeed = 1.0;

    int scoreState = 0;
    int flickStage = 0;
    private ElapsedTime flickTimer = new ElapsedTime();

    double shooterSpeed = 0.3;

    public void driveRobot(){
        double y = -gamepad1.left_stick_y * driveSpeed; // y up is negative
        double x = gamepad1.left_stick_x * driveSpeed;
        double rotationalPower = gamepad1.right_stick_x;


        double power = calculateDistance(x, y);
        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveMecanum(angle, power, rotationalPower);
    }

    public void runintake(){
        if(gamepad1.right_trigger > Constants.MINIMUM_TRIGGER_VALUE)
            motorIntake.setPower(0.8);

        else if (gamepad1.x)
            motorIntake.setPower(-0.8);
        else    motorIntake.setPower(0.0);
    }

    public void runLift(){
        if(gamepad1.dpad_up && !startScoring){
            //motorLift.setPower(0.1);
            motorLift.setTargetPosition(870);
        }
        else if (gamepad1.dpad_down && !startScoring){
            //motorLift.setPower(0.1);
            motorLift.setTargetPosition(0);
        }


        motorLift.setPower(Math.max((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 75.0), 1.0));
    }

    public void runShooter(){
        isShooterRunning = shooterOn.toggle(gamepad1.right_bumper);
        //if(flickStage < 6)
        startScoring = aPressed.toggle(gamepad1.a);
        //else    startScoring = false;
        isPowerShot = powerShot.toggle(gamepad1.y);

        if(isShooterRunning && !startScoring){
            if(isPowerShot) motorShooter.setPower(-0.29);
            else   motorShooter.setPower(-0.33);
        }
        else if(!startScoring){
            motorShooter.setPower(0.0);
        }

        if(startScoring){
            motorLift.setTargetPosition(870);
            if(isPowerShot)     motorShooter.setPower(-0.29);
            else    motorShooter.setPower(-0.33);
        }

        if(gamepad1.b && !isFlickerRunning){
            isFlickerRunning = true;
            flickTimer.reset();
            servoFlicker.setPosition(0.39); // first ring pushed out
            flickStage = 1;
        }
        if(flickStage == 1 && flickTimer.milliseconds() > 100){
            servoFlicker.setPosition(0.55); // flicker brought back in
            flickStage = 2;
        }
        if(flickStage == 2 && flickTimer.milliseconds() > 500){
            servoFlicker.setPosition(0.39); // second ring out
            flickStage = 3;
        }
        if(flickStage == 3 && flickTimer.milliseconds() > 600){
            servoFlicker.setPosition(0.55); // flicker brought back in
            flickStage = 4;
        }
        if(flickStage == 4 && flickTimer.milliseconds() > 1000){
            servoFlicker.setPosition(0.39); // second ring out
            flickStage = 5;
        }
        if(flickStage == 5 && flickTimer.milliseconds() > 1100){
            servoFlicker.setPosition(0.55); // flicker brought back in
            flickStage = 6;
        }
        if(flickStage == 6 && flickTimer.milliseconds() > 1500){
            //startScoring = false;
            motorLift.setTargetPosition(0);
            flickStage = 0;
            isFlickerRunning = false;
        }

        if(gamepad1.left_trigger > Constants.MINIMUM_TRIGGER_VALUE && !isFlickerRunning){
            //double servoPosition = map(gamepad1.left_trigger, Constants.MINIMUM_TRIGGER_VALUE,1,0.54,0.3);
            servoFlicker.setPosition(0.39);
        }else if(!isFlickerRunning){
            servoFlicker.setPosition(0.55);
        }
    }

    public void runArm(){

        if(gamepad2.left_stick_y > 0.15 || gamepad2.left_stick_y < -0.15)
        {
            motorWobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorWobble.setPower(Math.pow(gamepad2.left_stick_y,3) * 0.6);
        }
        else{
            motorWobble.setTargetPosition(motorWobble.getCurrentPosition());
            motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWobble.setPower(Math.max((motorWobble.getTargetPosition() - motorWobble.getCurrentPosition()) * (1 / 75.0), 1.0));
        }

    }

    public void runGrabber(){
        if(gamepad2.right_trigger > Constants.MINIMUM_TRIGGER_VALUE){
            servoGrabber.setPosition(0.5);
        }else{
            servoGrabber.setPosition(0);
        }
    }

    public void runDriveSpeed(){
        isSlowMode = driveSpeedToggle.toggle(gamepad1.left_bumper);
        if(isSlowMode) driveSpeed = 0.25;
        else driveSpeed = 1.0;
    }

    private double map(double value, double minInput, double maxInput, double minMappedOutput, double maxMappedOutput) {
        double valueDifference = maxInput - minInput;
        double percentValueDifference = (value - minInput) / valueDifference;
        double mappedDifference = maxMappedOutput - minMappedOutput;

        return percentValueDifference * mappedDifference + minMappedOutput;
    }

    void sendTelemetry(){
        telemetry.addData("Lift Ticks", motorLift.getCurrentPosition());
        telemetry.addData("Shoot Speed", motorShooter.getPower());
        telemetry.addData("Power Shot", isPowerShot);
        telemetry.addData("Flicker State", flickStage);
        telemetry.addData("Is Shooting", startScoring);
        telemetry.update();
    }
}

