package org.firstinspires.ftc.teamcode;
//package org.openftc.i2cdrivers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@I2cDeviceType()

public class CurtisMoveNormalizedNOLIMIT extends OpMode {

    //Motors
    private double MAXARMPOWER = 0.5;
    private double MAXSLIDEPOWER = 0.4;

    private double HangPower = 1;

//    public int arm = 0;
//    public int arm1 = 0;
//
//    public float armmove;
//    public int extend = 0;

    public Servo lancher = null;    

//    public float extendmove;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Arm, Slides = null;

    private double drive, strafe, turn, armPower, slidesPower = 0.0;

    ElapsedTime runtime = new ElapsedTime();
    // don't change
    double max;
    double[] speeds = new double[4];



    @Override
    public void init() {

        //Telemetry
        telemetry.addLine(">> Welcome :)");
        telemetry.update();

        // Initialize DcMotors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");

        Arm = hardwareMap.get(DcMotor.class, "AE");

        Slides = hardwareMap.get(DcMotor.class, "SE");

        lancher = hardwareMap.get(Servo.class, "PEW");
        //-------------------------------------------------------

        //set direction for motors not servos(servos do not need pos set)
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets em to back or forward
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        Slides.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    //Start function
    public void start() {
        //reset
        telemetry.clearAll();


        runtime.reset();
    }

    @Override
    public void loop() {

        // Set drive controls
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        // Set motor power
        speeds[0] = -drive + turn + strafe;
        speeds[1] = -drive - turn - strafe;
        speeds[2] = -drive + turn - strafe;
        speeds[3] = -drive - turn + strafe;

        max = Math.abs(speeds[0]);
        for(int i = 1; i < speeds.length; ++i) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; ++i) speeds[i] /= max;
        }




        // arm
        armPower = gamepad2.dpad_up ? MAXARMPOWER : gamepad2.dpad_down ? -MAXARMPOWER : 0;





        // slides
        slidesPower = gamepad2.a ? MAXSLIDEPOWER : gamepad2.b ? -MAXSLIDEPOWER : gamepad2.touchpad ? HangPower : 0;


        // Set motor powers to updated power

        if (gamepad1.right_bumper) {
            leftFrontDrive.setPower(speeds[0]/3);
            rightFrontDrive.setPower(speeds[1]/3);
            leftBackDrive.setPower(speeds[2]/3);
            rightBackDrive.setPower(speeds[3]/3);
        }
        else {
            leftFrontDrive.setPower(speeds[0]);
            rightFrontDrive.setPower(speeds[1]);
            leftBackDrive.setPower(speeds[2]);
            rightBackDrive.setPower(speeds[3]);
        }


        if(gamepad1.share)
        {
            lancher.setPosition(1);
        }

        if(gamepad1.left_bumper)
        {
            lancher.setPosition(0);

        }


        Arm.setPower(armPower);
        Slides.setPower(slidesPower);

        telemetry.addData("Extend Encoder Ticks", Slides.getCurrentPosition());

        telemetry.addData("Arm Encoder Ticks: ", Arm.getCurrentPosition());
    }



    @Override
    public void stop() {

        // Stop all motors if no input and if gamestop
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        Arm.setPower(0);
        Slides.setPower(0);
    }
}
