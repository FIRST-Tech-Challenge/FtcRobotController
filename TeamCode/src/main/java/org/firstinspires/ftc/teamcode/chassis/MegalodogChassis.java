package org.firstinspires.ftc.teamcode.chassis;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MegalodogChassis {
    public DcMotor FrontLeftWheel;
    public DcMotor FrontRightWheel;
    public DcMotor BackLeftWheel;
    public DcMotor BackRightWheel;
    private int WheelDiameter=104;    private LinearOpMode myOpMode;
    private int RPM = 435;
    private double ticksPerRevolution=384.5;
    public enum Direction {LEFT, CENTER, RIGHT}
    public enum Alliance {BLUE, RED}

    public static final int OneTileMM = 610;

    // This is the constructor for the class.  It takes a parameter for currentOp, which allows
    //   it to store and use the current op mode.  The four wheels are initialized here.
    public MegalodogChassis(LinearOpMode currentOp){

        // The op mode is important code provided by first.  It has the hardwareMap, sleep function,
        //   and telemetry functions.
        myOpMode = currentOp;
        HardwareMap hardwareMap = myOpMode.hardwareMap;

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
        FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)(WheelDiameter)
                BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)(WheelDiameter)
                BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)(WheelDiameter)
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

    public void ResetWheelConfig()
    {
        FrontRightWheel.setPower(0);
        BackRightWheel.setPower(0);
        FrontLeftWheel.setPower(0);
        BackLeftWheel.setPower(0);

        // Set the direction of the wheels.  Because of how the wheels are installed, one side
        //   has to be reverse.
        FrontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        BackRightWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        BackLeftWheel.setDirection(DcMotor.Direction.FORWARD);

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

    // This function strafes left.
    public void StrafeLeft(int mmToTarget, double VelocityPercentage, int WaitTime){
        StrafeRight(-mmToTarget, VelocityPercentage, WaitTime);
    }

    // this function strafes right
    public void StrafeRight(int mmToTarget, double VelocityPercentage, int WaitTime) {
        double TicksToTarget;
        double TicksPerSecond;

        TicksToTarget = (mmToTarget / (WheelDiameter * Math.PI)) * ticksPerRevolution;
        TicksPerSecond = ((VelocityPercentage * RPM) / 60) * ticksPerRevolution;
        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() + TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() - TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() - TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));

        // The (DcMotorEx) is called casting.  It says take the FrontLeftWheel, and while we know it's a DcMotor
        //   treat it like a DcMotorEx.  DcMotorEx has more functionality than DcMotor (such as setVelocity).  Not all hardware
        //   can be treated as a DcMotorEx, but ours can.  DcMotorEx inherits from DcMotor, so it has all the
        //   functionality of DcMotor plus more.  We should figure out if we can just declare these as DcMotorEx in
        //   the first place.  After tournaments.
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        while (myOpMode.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }

    /**
     * Move forward(+) or backwards(-) until reaching Position
     */
    public void MoveStraight(int mmToTarget, double VelocityPercentage, int WaitTime) {
        double TicksToTarget;
        double TicksPerSecond;

        TicksToTarget = (mmToTarget / ( * Math.PI)) * (ticksPerRevolution;
        TicksPerSecond = ((VelocityPercentage * RPM) / 60) * ticksPerRevolution;
        // myOpMode.telemetry.addData("ticksToTarget", TicksToTarget);
        //myOpMode.telemetry.update();
        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() + TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() + TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() + TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);
        while (myOpMode.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }

    public void RotateRight(int degree, double VelocityPercentage, int WaitTime){

        RotateLeft(-1*degree, VelocityPercentage, WaitTime);
    }

    public void RotateLeft(int degree, double VelocityPercentage, int WaitTime) {
        int mmToTarget;
        double TicksToTarget;
        double TicksPerSecond;

        // converts degree to a mm distance
        mmToTarget = degree * (560 / 90);
        // diamter of new robot wheels =
        // name is
        // uses the formula we've always had for rotation
        TicksToTarget = (mmToTarget / (WheelDiameter * Math.PI)) * ticksPerRevolution;
        TicksPerSecond = ((VelocityPercentage * RPM) / 60) * ticksPerRevolution;

        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() - TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() + TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() - TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        while (myOpMode.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }



