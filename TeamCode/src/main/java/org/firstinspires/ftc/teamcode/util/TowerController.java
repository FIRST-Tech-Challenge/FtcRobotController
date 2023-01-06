package org.firstinspires.ftc.teamcode.util;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            //hi. you found me. -SECRET COMMENT
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TowerController
{
    // Screw Fields
    private DcMotor screw;
    private TouchSensor highSensor;
    private TouchSensor lowSensor;
    private int screwLevel = 0;

    private boolean uBarY;
    private boolean uBarX;
    private boolean uBarB;
    private boolean screwA;


    // Ubar field
    private DcMotor uBar;

    // Intake Fields
    private DcMotor intake;
//    private Servo intake2;
//    private TouchSensor intakeSensor;

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

        // Make sure Screw will go the right way
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
        // sets power
        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
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
        // Sets target position
        screwLevel -= screwTarget;
        // Sets direction
        screw.setDirection(DcMotor.Direction.REVERSE);
        screw.setTargetPosition(screwLevel);
        // sets run mode
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // sets power
        screw.setPower(-speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (lowSensor.isPressed())))
        {
            // Stops if sensor is true
            if (lowSensor.isPressed())
            {
                telemetry.addData("LowSensor is pressed", "");
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

    public void handleScrewLevelSet(Gamepad gamepad, Telemetry telemetry)
    {
        // Multiply all values by .71
        //4 button screw position set
        if (gamepad.dpad_up)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(3790);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
        }
        else if (gamepad.dpad_left)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(-5);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
        }
        else if (gamepad.dpad_right)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(4675);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
        }
        else if (gamepad.dpad_down)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(1600);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
        }

    }

    public void handleUBarLevelSet(Gamepad gamepad, Telemetry telemetry)
    {
        uBar.setDirection(DcMotorSimple.Direction.REVERSE);
        //4 button Ubar position set
        // High Junction
        if (gamepad.y)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-3109);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(1);
        }

        // Middle Junction
        else if (!gamepad.start && gamepad.b)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-1680);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(1);
        }

        // Low Junction
        else if (gamepad.x)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-3214);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(-1);
        }

        // Pickup Junction
        else if (gamepad.a)
        {
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(532);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(1);
        }


        if (gamepad.left_trigger > 0.5)
        {
            uBar.setPower(0);
            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void screwAnlogControler(Gamepad gamepad, Telemetry telemetry)
    {
        double screwPower;
        double driveScrew = gamepad.left_stick_y;
        screwPower = Range.clip(driveScrew, -1, 1);
//        if ((screwPower < 0 && !highSensor.isPressed()) || screwPower > 0 && !lowSensor.isPressed())
//        {
//            screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            screw.setPower(-screwPower);
//        }
//        else
//        {
////            screw.setPower(0)
////            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }

        //Stick is active, within bounds
        //Stick is active, out of bounds
        //Stick is not active

        if (Math.abs(screwPower) > 0)
        {
            if (lowSensor.isPressed() && screwPower > 0)
            {
                //Out of bounds.  Kill power!
                screw.setPower(0);
            }
            else if (highSensor.isPressed() && screwPower < 0)
            {
                screw.setPower(0);
            }
            else
            {
                screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                screw.setPower(-screwPower);
            }
        }
        else if (screw.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            //Ensure motor is not moving.
            screw.setTargetPosition(screw.getCurrentPosition());
            screw.setPower(0);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (screw.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        {
            //Do nothing
        }
    }

    private void driveAll(Gamepad gamepad, Telemetry telemetry)
    {
        //3885
        //4 button screw position set
        if (gamepad.y)
        {
            uBarY = true;
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(3790);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
        }

        if (uBarY && screw.getCurrentPosition() > 3750)
        {
            uBarY = false;
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-3109);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(0.5);
        }

        if (gamepad.x)
        {
            uBarX = true;
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(-5);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
        }

        if (screw.getCurrentPosition() > 3750 && uBarX)
        {
            uBarX = false;
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-1680);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(0.5);
        }

        if (gamepad.b && !gamepad.start)
        {
            uBarB = true;
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(4675);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);

        }

        if (screw.getCurrentPosition() > 3750 && uBarB)
        {
            uBarB = false;
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(-1680);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(0.5);
        }

        if (gamepad.a)
        {
            screwA = true;
            uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uBar.setTargetPosition(532);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uBar.setPower(0.5);
        }
        if (screwA && uBar.getCurrentPosition() < 500)
        {
            screwA = false;
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            screw.setTargetPosition(1600);
            screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            screw.setPower(1);
        }
    }

    public void uBarAnlogControler(Gamepad gamepad, Telemetry telemetry)
    {
        double uBarPower;
        double driveUBar = gamepad.right_stick_y;
        uBarPower = Range.clip(driveUBar, -1, 1);
//        if (!(uBarPower == 0))
//        {
//            uBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            uBar.setPower(uBarPower);
//        }
//        else
//        {
// //           uBar.setPower(0);
// //           uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }

//        telemetry.addData("UBar stick", "&f", uBarPower);

        if (Math.abs(uBarPower) > 0)
        {
            //stick is active.  Break out of RunTo mode and run without encoder
            uBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uBar.setPower(-uBarPower);
        }
        else if (uBar.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            //Ensure motor is not moving. Will "active lock" to this location.
            uBar.setTargetPosition(uBar.getCurrentPosition());
            uBar.setPower(1);
            uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (uBar.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        {
            //Do nothing, stick is idle
        }
    }

    public void handleGamepad(Gamepad gamepad, Telemetry telemetry)
    {
        //Screw methods
        handleScrewLevelSet(gamepad, telemetry);
        screwAnlogControler(gamepad, telemetry);

        //Ubar Methods
        handleUBarLevelSet(gamepad, telemetry);
        uBarAnlogControler(gamepad, telemetry);

        // Intake Method
        handleIntake(gamepad);

        //Screw and Ubar ticks printout or breaking method
        telemetryOutput(telemetry);
    }

    public void telemetryOutput(Telemetry telemetry)
    {
        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
        telemetry.addData("Ubar  ticks = ", "%d", uBar.getCurrentPosition());

        telemetry.update();
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    //hi. you found me. -SECRET COMMENT