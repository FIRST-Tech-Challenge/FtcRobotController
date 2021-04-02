package org.firstinspires.ftc.team8923_2020;



abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;
    private boolean prevClawPosition = false;

    private Toggle slowMode = new Toggle();

    boolean isliftUp = false;
    boolean isShooterRunning = false;

    double shooterSpeed = 0.3;

    public void driveRobot()
    {
        double y = -gamepad1.left_stick_y; // y up is negative
        double x = gamepad1.left_stick_x;
        double rotationalPower = gamepad1.right_stick_x;


        double power = calculateDistance(x, y);
        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveMecanum(angle, power, rotationalPower);
    }

    public void runintake(){
        if(gamepad1.right_trigger > Constants.MINIMUM_TRIGGER_VALUE)
            motorIntake.setPower(0.95);

        else if (gamepad1.left_trigger > Constants.MINIMUM_TRIGGER_VALUE)
            motorIntake.setPower(-0.95);
        else    motorIntake.setPower(0.0);
    }

    public void runLift(){

        if(gamepad1.dpad_up){
            motorLift.setTargetPosition(870);
        }
        else if (gamepad1.dpad_down){
            motorLift.setTargetPosition(0);
        }
        motorLift.setPower(Math.max((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 75.0), 1.0));
    }

    public void runShooter(){
        if(gamepad1.right_bumper && !isShooterRunning){
            motorShooter.setPower(- 0.8);
            isShooterRunning = true;
        }
        else if(gamepad1.left_bumper && isShooterRunning){
            motorShooter.setPower(0.0);
            isShooterRunning = false;
        }
        if(gamepad1.a){
            servoFlicker.setPosition(0.55);
            //servoFlicker.wait(300);
        }
        if(gamepad1.b){
            servoFlicker.setPosition(0.3);
        }
    }

    void sendTelemetry(){
        telemetry.addData("Lift Ticks", motorLift.getCurrentPosition());
        telemetry.addData("Shoot Speed", motorShooter.getPower());
        telemetry.update();
    }
}

