package org.firstinspires.ftc.teamcode;
//package org.openftc.i2cdrivers;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@TeleOp
@I2cDeviceType()

public class Move extends OpMode {
    //Motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor Arm = null;

    private DcMotor Extend = null;

    //bools
    public boolean Endgametrue = false;
    public boolean Parktrue = false;

    //ElapsedTime Runtime
    public float armmove;
    public float extendmove;
    public float extendpower;
    public float armPower = 0;

    public int arm = 0;
    public int extend = 0;
    ElapsedTime runtime = new ElapsedTime();

//    static void sleep(int LongMilliseconds) {
//        try {
//            Thread.sleep(LongMilliseconds);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//    }

    @Override
    public void init() {

        //Telemetry
        telemetry.addLine(">> Press Start Button");
        telemetry.update();



        // Initialize DcMotors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");

        Arm = hardwareMap.get(DcMotor.class, "ARM");

        Extend = hardwareMap.get(DcMotor.class, "EX");

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Sets em to back or forward
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        Extend.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    //Start fuction
    public void start() {
        //restes all telemitry
        telemetry.clearAll();
        Endgametrue = false;
        Parktrue = false;
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower; // Declare motor power variables

        // Set drive controls
        double drive = -gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // Set motor power
        leftFrontPower = Range.clip(drive + turn + strafe, -1, 1);
        rightFrontPower = Range.clip(drive - turn - strafe, -1, 1);
        leftBackPower = Range.clip(drive + turn - strafe, -1, 1);
        rightBackPower = Range.clip(drive - turn + strafe, -1, 1);

        // Telemetry
        telemetry.addData("Speed: ", (leftFrontPower + leftBackPower + rightBackPower + rightFrontPower) / 4);
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y: ", -gamepad1.left_stick_y);
        telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y: ", -gamepad1.right_stick_y);
        telemetry.addData("Lift Encoder Ticks: ", Arm.getCurrentPosition());
        telemetry.addData("Turret Encoder Ticks", Extend.getCurrentPosition());
        telemetry.addData("arm", arm);
        telemetry.update();


        //backup


        // Slow movement + Fast movement


//        else if(gamepad1.a)
//        {
//            armmove = armmove + 0.05f;
//            armPower = armmove;
//
//        }
//        else if(gamepad1.b)
//        {
////            if(armmove != 10)
////            {
//                armmove = armmove - 0.05f;
//                armPower = armmove;
////            }
//
//
//        }

        if(gamepad1.right_bumper) {
            leftFrontPower /= 4;
            leftBackPower /= 4;
            rightFrontPower /= 4;
            rightBackPower /= 4;
        }

        if(gamepad1.dpad_up)
        {

            armmove = -1f;

            armPower = armmove;
            if(Arm.getCurrentPosition() > -2542)
            {
                Arm.setTargetPosition(arm);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(2);
                arm = arm - 5;
            }

        }
        else if(gamepad1.dpad_down)
        {

            armmove = 1f;


            if(Arm.getCurrentPosition() < -1)
            {
                arm = arm + 5;
                armPower = armmove;

                Arm.setTargetPosition(arm);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(2);
            }
        }
        else
        {
            armPower = 0;
        }
        if(gamepad1.a)
        {
            if(Extend.getCurrentPosition() > -3235)
            {
                Extend.setTargetPosition(extend);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extend.setPower(-2);


                extend = extend - 5;
            }


            extendmove = 1f;


            extendpower = extendmove;
        }
        else if(gamepad1.b)
        {


            if(Extend.getCurrentPosition() < -1)
            {
                Extend.setTargetPosition(extend);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extend.setPower(2);
                extend = extend + 5;
                extendmove = -1f;
            }


            extendpower = extendmove;
        }
        else
        {
            extendpower = 0;
        }



        // Set motor powers to updated power
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        Arm.setPower(armPower);
        Extend.setPower(extendpower);




    }

    @Override
    public void stop() {
        // Stop all motors if no input
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        Arm.setPower(0);
        Extend.setPower(0);
    }
}

