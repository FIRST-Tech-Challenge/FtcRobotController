package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Config_Rotation extends LinearOpMode {

    // declare variables here
    //
    private DcMotor FrontRightWheel;
    private DcMotor FrontLeftWheel;
    private DcMotor BackRightWheel;
    private DcMotor BackLeftWheel;
    @Override
    public void runOpMode() {

        //    Do initialization things here
        InitWheels();

        waitForStart();


        while (opModeIsActive()) {
            // do op mode things here
            manageDriverControls();

        }
    }

    private void manageDriverControls()
    {
        if(gamepad1.triangle)
        {
            // do something
            RotateLeft(300, 0.4, 1000);
        }
        if(gamepad1.square)
        {
            RotateLeftDegree(90, 0.4, 1000);
        }

    }

    private void InitWheels() {

        HardwareMap hardwareMap = this.hardwareMap;

        // This gets the devices from the configuration on the robot.
        //    This call basically says "Get me the thing called FrontRightWheel from the
        //    configuration, and trust me it can be mapped to the DCMotor class.  If it's
        //    not a motor, then code later on will throw errors when it tries to do motor
        //    things with a non motor.
        FrontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        BackRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");
        FrontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        BackLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");

        // Set the direction of the wheels.  Because of how the wheels are installed, one side
        //   has to be reverse.
        FrontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        BackRightWheel.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        // > Set motors' ZeroPower behavior
        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // > Clear Encoders of prior data
        FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftWheel.setTargetPosition(0);
        FrontRightWheel.setTargetPosition(0);
        BackLeftWheel.setTargetPosition(0);
        BackRightWheel.setTargetPosition(0);

        // > Set some motors' modes different from RUN_WITHOUT_ENCODER (default)
        FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void RotateLeft(int mmToTarget, double VelocityPercentage, int WaitTime) {
        //   int mmToTarget;
        double TicksToTarget;
        double TicksPerSecond;

        // converts degree to a mm distance
        //    mmToTarget = degree * (560 / 90);

        // uses the formula we've always had for rotation
        TicksToTarget = (mmToTarget / (96 * Math.PI)) * 537.7;
        TicksPerSecond = ((VelocityPercentage * 312) / 60) * 537.7;
        telemetry.addData("RotateLeft mmToTarget", mmToTarget);
        telemetry.addData("RotateLeft TicksToTarget", TicksToTarget);
        telemetry.addData("RotateLeft TicksPerSecond", TicksPerSecond);
        telemetry.update();

        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() - TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() + TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() - TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        while (this.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        this.sleep(WaitTime);
    }

    public void RotateLeftDegree(int degree, double VelocityPercentage, int WaitTime) {
        double mmToTarget;
        double TicksToTarget;
        double TicksPerSecond;

        // converts degree to a mm distance
        //   mmToTarget = degree * (560 / 90);
        mmToTarget = degree * ((double)300 / 90);
        // uses the formula we've always had for rotation
        TicksToTarget = (mmToTarget / (96 * Math.PI)) * 537.7;
        TicksPerSecond = ((VelocityPercentage * 312) / 60) * 537.7;
        telemetry.addData("RotateLeft mmToTarget", mmToTarget);
        telemetry.addData("RotateLeft TicksToTarget", TicksToTarget);
        telemetry.addData("RotateLeft TicksPerSecond", TicksPerSecond);
        telemetry.update();

        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() - TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() + TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() - TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        while (this.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        this.sleep(WaitTime);
    }
}
