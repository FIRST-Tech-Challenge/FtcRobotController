package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "EchoOp")
public class EchoOp extends OpMode {
    private Motor driveLeft, driveRight;
    private Motor intake1, intake2;
    private Motor flywheel;
    private Motor hitter;
    private double speedModifier = 1;
    private double intakeSpeed = 0;
    private double flyWheelToggle = 0;
    private double flyWheelDirection = -1;
    private boolean pressedB = false;

    @Override
    public void init() {
        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);

        flywheel = new Motor(hardwareMap, "fw");
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setFeedforwardCoefficients(0, 0.03);
        flywheel.setVeloCoefficients(40, 0.1, 0);

        hitter = new Motor(hardwareMap, "h");
        hitter.setRunMode(Motor.RunMode.PositionControl);
        hitter.setPositionCoefficient(0.004);
        hitter.setPositionTolerance(13.6);
    }

    @Override
    public void loop() {



        double leftSpeed = -gamepad1.left_stick_y;
        double rightSpeed = gamepad1.right_stick_y;
        if(gamepad1.right_trigger >= 0.1) {
            intakeSpeed = 0.7;
        } else if(gamepad1.left_trigger >= 0.1) {
            intakeSpeed = -0.7;
        } else {
            intakeSpeed = 0;
        }

        if(gamepad1.b) {
            pressedB = true;
        } else if(pressedB) {
            if(flyWheelToggle != 0) {
                flywheel.setRunMode(Motor.RunMode.RawPower);
                flyWheelToggle = 0;
            } else {
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
                flywheel.setFeedforwardCoefficients(0, 0.03);
                flywheel.setVeloCoefficients(40, 0.1, 0);
                flyWheelToggle = 1;
            }
            pressedB = false;
        }

        if(gamepad1.left_bumper) {
            hitter.setTargetPosition(150);
            if(!hitter.atTargetPosition()) {
                hitter.set(0.7);
            } else {
                hitter.set(0);
            }
        } else {
            hitter.setTargetPosition(0);
            if(!hitter.atTargetPosition()) {
                hitter.set(0.7);
            } else {
                hitter.set(0);
            }
        }

        driveLeft.set(leftSpeed);
        driveRight.set(rightSpeed);

        intake1.set(intakeSpeed);
        intake2.set(intakeSpeed);

        flywheel.set(flyWheelToggle * flyWheelDirection * 0.42);


        telemetry.addData("Flywheel Speed", flywheel.get());
        telemetry.addData("Left Speed", leftSpeed);
        telemetry.addData("Right Speed", rightSpeed);
        telemetry.addData("Intake Speed", intakeSpeed);


    }
}