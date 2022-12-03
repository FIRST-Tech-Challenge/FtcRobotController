package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TowerController
{

    //setup variables, motors, and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor screw;
    private DcMotor uBar;
    private Servo intake2;
    private DcMotor intake;
    private TouchSensor highSensor;
    private TouchSensor lowSensor;
    public boolean raiseTower;
    public boolean intakePick = false;
    private int uBarLevel;
    private int screwLevel;
    private int intakeLevel;
    private int previousLevel;
    private int level = 1;
    static final double     COUNTS_PER_MOTOR    = 384.5;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION);


    public TowerController (HardwareMap hardwareMap)
    {

        //Setup motors
        highSensor = hardwareMap.get(TouchSensor.class, "highSensor");
        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");
        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");

        intake2 = hardwareMap.get(Servo.class, "intake2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        screw.setDirection(DcMotor.Direction.FORWARD);
        uBar.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        //Setup sensors NOT NEEDED
//        highSensor = hardwareMap.get(DigitalChannel.class, "highSensor");
//        DigitalChannel lowSensor = hardwareMap.get(DigitalChannel.class, "lowSensor");
//        highSensor.setMode(DigitalChannel.Mode.INPUT);
//        lowSensor.setMode(DigitalChannel.Mode.INPUT);

        //setup encoder
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        screwLevel = 1000;
        screw.setDirection(DcMotor.Direction.REVERSE);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        screw.setPower(-0.1);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwLevel)) || (screw.isBusy() && (lowSensor.isPressed())))
        {
            if (lowSensor.isPressed())
            {
                break;
            }
        }

        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setPower(0);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
    }

//    public void handleScrew() {
//        if(raiseTower){
//            if(highSensor.getState()){
//                screw.setPower(0);
//            }
//            else{
//                screw.setPower(1);
//            }
//        }
//        else{
//            if(lowSensor.getState()){
//                screw.setPower(0);
//            }
//            else{
//                screw.setPower(-1);
//            }
//        }
//    }

//

    private void driveUBar(double uBarTarget, double speed, Telemetry telemetry)
    {
        uBarLevel += uBarTarget;
        uBar.setTargetPosition(uBarLevel);
        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uBar.setPower(speed);
        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
        telemetry.update();
        while (uBar.isBusy() && uBar.getCurrentPosition() <= uBarTarget)
        {
            telemetry.addData("UBar ticks = ", "%d", uBar.getCurrentPosition());
            telemetry.update();
            uBar.setPower(speed);
        }
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setPower(0);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     *
     * @param screwTarget
     * @param speed
     * @param telemetry
     */
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


    /**
     *
     * @param screwTarget
     * @param speed
     * @param telemetry
     */
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
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
/*
        if (highSensor.isPressed())
        {
            screw.setDirection(DcMotor.Direction.FORWARD);
            screwTarget = 476;
            screwLevel -= screwTarget;
            while (screw.isBusy() && screw.getCurrentPosition() <= screwTarget)
            {
                screw.setTargetPosition(screwLevel);
                screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                screw.setPower(speed);
            }
            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        screwLevel = 0;

*/

    }

    public void handleIntake (double intakeTarget, double speed, Telemetry telemetry)
    {

        if(intakePick){
            intake2.setPosition(0.1);
            intake2.getPosition();
        }
        else{
            intake2.setPosition(0);
            intake2.getPosition();
        }

        // Wheels
        if(intakePick)
        {
            intakeLevel -= intakeTarget;
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setTargetPosition(screwLevel);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            screw.setPower(speed);
            while ((intake.isBusy() && (intake.getCurrentPosition() <= intakeTarget)))
            {
                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
                telemetry.update();
            }

            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            screwLevel = 0;
        }

        if (!intakePick)
        {
            intakeLevel -= intakeTarget;
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setTargetPosition(screwLevel);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            screw.setPower(speed);
            while ((intake.isBusy() && (intake.getCurrentPosition() <= intakeTarget)))
            {
                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
                telemetry.update();
            }

            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            screwLevel = 0;
        }
    }

    public void screwDriveToLevelUp(int screwTarget, double speed, Telemetry telemetry)
    {
        screwLevel -= screwTarget;
        screw.setDirection(DcMotor.Direction.FORWARD);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();

        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();


            if (lowSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void screwDriveToLevelDown(int screwTarget, double speed, Telemetry telemetry)
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
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void switchLevel(Telemetry telemetry)
    {
        switch (level)
        {
            case 1:
                if (level < previousLevel)
                {
                    screwDriveToLevelDown(screwLevel - 0, 0.50, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(0 - screwLevel, 0.50, telemetry);
                }
                break;

            case 2:
                if (level < previousLevel)
                {
                    screwDriveToLevelDown(screwLevel - 1000, 0.50, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(1000 - screwLevel, 0.50, telemetry);
                }
                break;

            case 3:
                if (level < previousLevel)
                {
                    screwDriveToLevelDown(screwLevel - 2000, 0.50, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(2000 - screwLevel, 0.50, telemetry);
                }
                break;

            case 4:
                if (level < previousLevel)
                {

                    screwDriveToLevelDown(screwLevel - 3000, 0.50, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(3000 - screwLevel, 0.50, telemetry);
                }
                break;

            default:
                break;
        }
    }

    public void handleGamepad(Gamepad gamepad, Telemetry telemetry)
    {

        //Screw
//        if(gamepad.dpad_up)
//        {
//            driveScrewUp(2000, 0.75, telemetry);
//        }
//        if(gamepad.dpad_down)
//        {
//            driveScrewDown(2000, 0.75, telemetry);
//        }

        if (gamepad.dpad_up && level < 4)
        {
            previousLevel = level;
            level++;
            switchLevel(telemetry);
        }
        if (gamepad.dpad_down && level > 1)
        {
            previousLevel = level;
            level--;
            switchLevel(telemetry);
        }

        //U Bar
        int num = 0;
//        if(gamepad.b)
//        {
//            gamepad.b = false;
//            driveUBar(	330.06875, 0.2, telemetry);
//            //60 degrees
//        }
        if(gamepad.a)
        {
            gamepad.a = false;
            driveUBar(	165.034375, 0.2, telemetry);
            //30 degrees
        }
        if(gamepad.x)
        {
            gamepad.x = false;
            driveUBar(	-330.06875, 0.2, telemetry);
            //60 degrees
        }
        if(gamepad.y)
        {
            gamepad.y = false;
            driveUBar(	-165.034375, 0.2, telemetry);
            //30 degrees
        }

        //intake wheel
        if (gamepad.left_bumper)
        {

        }
        if (gamepad.right_bumper)

        //Intake scissor cone
        if(gamepad.right_bumper)
        {
            intakePick = true;
            handleIntake(50, 0.1, telemetry);
        }
        if(gamepad.left_bumper)
        {
            intakePick = false;
            handleIntake(50, 0.1, telemetry);
        }

    }

}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //hi. you found me. -SECRET COMMENT