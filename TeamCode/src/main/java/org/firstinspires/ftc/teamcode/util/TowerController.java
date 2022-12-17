package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TowerController
{

    //setup variables, motors, and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor uBar;
    private int uBarLevel;
    private double uBarPower;

    private DcMotor screw;
    private int screwLevel = 0;
    private TouchSensor highSensor;
    private TouchSensor lowSensor;
    private int previousLevel;
    private int level = 1;

//    private Servo intake2;
    private DcMotor intake;
//    private int intakeLevel;
//    private TouchSensor intakeSensor;
    public int intakePick = 0;

    public TowerController (HardwareMap hardwareMap , Telemetry telemetry)
    {

        //Setup motors
        highSensor = hardwareMap.get(TouchSensor.class, "highSensor");
        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");
        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");

//        intake2 = hardwareMap.get(Servo.class, "intake2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        screw.setDirection(DcMotor.Direction.FORWARD);
        uBar.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Commit out if no intake sensor
//        intakeSensor = hardwareMap.get(TouchSensor.class, "intakeSensor");


        //setup encoder
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // init screw to bottom level
        driveScrewUp(500,0.5, telemetry);
        driveScrewDown(10000, 0.5, telemetry);
        screw.setDirection(DcMotor.Direction.REVERSE);
    }

    private void driveScrewUp(double screwTarget, double speed, Telemetry telemetry)
    {
        // Sets target position
        screwLevel -= screwTarget;
        // Sets direction
        screw.setDirection(DcMotor.Direction.FORWARD);
        screw.setTargetPosition(screwLevel);
        // sets run mode
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();
        // sets power
        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
            // shows how many ticks
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();

            // Stops if sensor is true
            if (highSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();
        }

        // breaks the motor
        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
    }

    private void driveScrewDown(double screwTarget, double speed, Telemetry telemetry)
    {
        screwLevel -= screwTarget;
        screw.setDirection(DcMotor.Direction.REVERSE);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();

        screw.setPower(-speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (lowSensor.isPressed())))
        {
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();


            if (lowSensor.isPressed())
            {
                telemetry.addData("LowSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
    }

    public void handleScrewLevelSet(Gamepad gamepad, Telemetry telemetry)
    {
        //4/3 button screw position set
        if (gamepad.dpad_up)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(4270);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(-1);
            while (screw.isBusy())
            {
                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
                telemetry.update();
            }
            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
        }
        else if (gamepad.dpad_left)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(303);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(-1);
            while (screw.isBusy())
            {
                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
                telemetry.update();
            }
            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
        }
        else if (gamepad.dpad_right)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(4557);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
            while (screw.isBusy())
            {
                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
                telemetry.update();
            }
            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
        }
        else if (gamepad.dpad_down)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(809);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(-1);
            while (screw.isBusy())
            {
                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
                telemetry.update();
            }
            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
        }

    }
    public void handleUBarLevelSet(Gamepad gamepad, Telemetry telemetry)
    {
        uBar.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
        telemetry.update();

        //4 button Ubar position set

        // High Juction
        if (gamepad.y)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-4370);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(0.5);
            while (uBar.isBusy())
            {
                telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                telemetry.update();
            }
            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            uBar.setPower(0);
        }

        // Middle Juction
        else if (!gamepad.start && gamepad.b)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-4315);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(0.5);
            while (uBar.isBusy())
            {
                telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                telemetry.update();
            }
            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            uBar.setPower(0);
        }

        // Low Juction
        else if (gamepad.x)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-2158);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(-0.5);
            while (uBar.isBusy())
            {
                telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                telemetry.update();
            }
            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            uBar.setPower(0);
        }

        // Pickup Juction
        else if (gamepad.a)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(32);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(0.5);
            while (uBar.isBusy())
            {
                telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                telemetry.update();
            }
            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            uBar.setPower(0);
        }
    }

    public void handleIntake(Gamepad gamepad)
    {
        if (gamepad.right_bumper)
        {
            intake.setPower(1);
        }
        else if (gamepad.left_bumper)
        {
            intake.setPower(-1);
        }
        else
        {
            intake.setPower(0);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void handleGamepad(Gamepad gamepad, Telemetry telemetry)
    {

        double screwPower;
        screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double driveScrew = gamepad.left_stick_y;
        screwPower = Range.clip(driveScrew, -1, 1);
        if ((screwPower < 0 && !highSensor.isPressed()) || screwPower > 0 && !lowSensor.isPressed())
        {
            screw.setPower(-screwPower);
        }
        else
        {
            screw.setPower(0);
            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        double uBarPower;
        uBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double driveUBar = gamepad.right_stick_y;
        uBarPower = Range.clip(driveUBar, -1, 1);
        if (!(uBarPower == 0))
        {
            uBar.setPower(uBarPower);
        }
        else
        {
            telemetry.addData("breaking", "", "");
            uBar.setPower(0);
            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        handleIntake(gamepad);
        handleScrewLevelSet(gamepad, telemetry);
         handleUBarLevelSet(gamepad, telemetry);

        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
        telemetry.addData("Speed", uBarPower);
        telemetry.addData("Ubar ticks = ", "%d", uBar.getCurrentPosition());
        telemetry.update();

    }
}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    //hi. you found me. -SECRET COMMENT