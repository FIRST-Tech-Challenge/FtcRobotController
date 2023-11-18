package org.firstinspires.ftc.teamcode;
//package org.openftc.i2cdrivers;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
@I2cDeviceType()

public class Move extends OpMode {
    //Motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    //public Servo fire = null;


    private DcMotor Arm = null;

    private DcMotor Extend = null;

    //bools
    public boolean Endgametrue = false;
    public boolean Parktrue = false;

    public boolean Grab = false;

    //floats Runtime
    public float armmove;
    public float extendmove;
    public float extendpower;
    public float armPower = 0;

    public float movedown;

    //ints
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
        rightBackDrive = hardwareMap.get(DcMotor.class, "RF");
        //fire = hardwareMap.get(Servo.class, "Fire");
        Arm = hardwareMap.get(DcMotor.class, "AE");

        Extend = hardwareMap.get(DcMotor.class, "SE");
        //-------------------------------------------------------

        //set direction for motors not servos(servos do not need pos set)
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Sets em to back or forward
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        Extend.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    //Start fuction
    public void start() {
        //reset
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
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn =  gamepad1.right_stick_x;

        // Set motor power
        leftFrontPower = Range.clip(drive - turn - strafe, -0.6, 0.6);
        rightFrontPower = Range.clip(drive + turn + strafe, -0.6, 0.6);
        leftBackPower = Range.clip(drive - turn + strafe, -0.6, 0.6);
        rightBackPower = Range.clip(drive + turn - strafe, -0.6, 0.6);



        // Telemetry
        telemetry.addData("Speed: ", (leftFrontPower + leftBackPower + rightBackPower + rightFrontPower) / 4);
        telemetry.addData("Left Stick X Input: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y Input: ", -gamepad1.left_stick_y);
        telemetry.addData("Right Stick X Input: ", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y Input: ", -gamepad1.right_stick_y);
        telemetry.addData("Arm Encoder Ticks: ", Arm.getCurrentPosition());
        telemetry.addData("Extend Encoder Ticks", Extend.getCurrentPosition());
        telemetry.addData("arm", arm);
        telemetry.update();
        //------------------

        //slow speed
        if(gamepad1.right_bumper) {
            leftFrontPower /= 3.5;
            leftBackPower /= 3.5;
            rightFrontPower /= 3.5;
            rightBackPower /= 3.5;
        }


        //armmovepower
        if(gamepad2.dpad_up)
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
        else
        {
            armPower = 0;
        }


        //    |
        //arm V
         if(gamepad2.dpad_right) {

            arm = arm + 5;
            armPower = armmove;

            Arm.setTargetPosition(arm);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(2);
        }
         else
         {
             armPower = 0;
         }

        if(gamepad2.dpad_left) {

            arm = arm - 5;
            armPower = armmove;

            Arm.setTargetPosition(arm);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(2);
        }
        else
        {
            armPower = 0;
        }


        if(gamepad2.dpad_down)
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


        if(gamepad1.share) {
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Extend.setPower(2);

            Arm.setTargetPosition(0);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(2);
        }
        if(gamepad2.x)
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
        else if(gamepad2.y)
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
        else if(gamepad2.a || Grab )
        {
            Grab = true;
            movedown++;

            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Extend.setPower(2);

            if(Extend.getCurrentPosition() == 0)
            {
                Grab = false;
            }




         }
        else if(gamepad1.left_bumper)
        {
            //fire.setPosition(0.0);

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

        // Stop all motors if no input and if gamestop
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        Arm.setPower(0);
        Extend.setPower(0);
    }
}

