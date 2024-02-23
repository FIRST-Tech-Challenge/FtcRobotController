package org.firstinspires.ftc.teamcode.drivecode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp
public class GoBilda extends LinearOpMode {


    private DcMotor armRotate;
    private DcMotor armBrace;
    private DcMotor armExt;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo servoRight;
    private Servo servoLeft;
    private RevBlinkinLedDriver revBlinkin;
    private Servo servoDrone;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //Initialize Variables
        //Tracks rotation of the arm's motor
        int rotation;
        //Tracks the extension of the arm
        int ext;
        //X and Y values of stick inputs to compile drive outputs
        double Ry;
        double Rx;
        double Ly;
        double Lx;
        //Value for armLock
        boolean armLocked;

        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        revBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "revBlinkin");
        servoDrone = hardwareMap.get(Servo.class, "servoDrone");

        //Sets variables to 0 on initialization
        rotation = 0;
        ext = 0;
        armLocked = false;
        waitForStart();

        if (opModeIsActive()) {

            //Sets behaviors and modes for motors
            //ArmExtension and ArmRotate are set to brake when receiving zero power
            //Arm Extension is set to run using encoder outputs and inputs
            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBrace.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armBrace.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive()) {

                //Assigns variables to inputs
                //Ext and Rotation are set to receive inputs from encoders
                ext = armExt.getCurrentPosition();
                rotation = armRotate.getCurrentPosition();
                //Multipliers are applied to X and Y and they are tied to sticks on the game pads.
                Ly = gamepad1.left_stick_y;
                Rx = gamepad1.right_stick_x;
                Lx = gamepad1.left_stick_x;
                Ry = gamepad1.right_stick_y;

                if (gamepad1.right_trigger > .5) {
                    if (Ly > 0.2 || Ly < -0.2 || Rx > .02 || Rx < -.02) {
                        // Waiting for Left Joystick to Move & Scaling the Motor Value
                        frontLeftMotor.setPower((Ly+-Rx)*0.25);
                        frontRightMotor.setPower((-Ly+-Rx)*0.25);
                        backLeftMotor.setPower((Ly+-Rx)*0.25);
                        backRightMotor.setPower((-Ly+-Rx)*0.25);
                    } else if (gamepad1.left_bumper) {
                        frontLeftMotor.setPower(-.3);
                        frontRightMotor.setPower(.3);
                        backLeftMotor.setPower(.3);
                        backRightMotor.setPower(-.3);
                    } else if (gamepad1.right_bumper) {
                        frontLeftMotor.setPower(.3);
                        frontRightMotor.setPower(-.3);
                        backLeftMotor.setPower(-.3);
                        backRightMotor.setPower(.3);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                } else {
                    if (Ly > 0.2 || Ly < -0.2 || Rx > 0.2 || Rx < -0.2) {
                        frontLeftMotor.setPower(Ly+-Rx);
                        frontRightMotor.setPower(-Ly+-Rx);
                        backLeftMotor.setPower(Ly+-Rx);
                        backRightMotor.setPower(-Ly+-Rx);
                    } else if (gamepad1.left_bumper) {
                        frontLeftMotor.setPower(-1);
                        frontRightMotor.setPower(1);
                        backLeftMotor.setPower(1);
                        backRightMotor.setPower(-1);
                    } else if (gamepad1.right_bumper) {
                        frontLeftMotor.setPower(1);
                        frontRightMotor.setPower(-1);
                        backLeftMotor.setPower(-1);
                        backRightMotor.setPower(1);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                }

                //Arm rotation controls
                //Rotates up when Right Bumper is pressed
                //Rotates down when Left Bumper is pressed
                //Otherwise power is set to 0 (BRAKE)

                //if(armLocked == false){

                if (armLocked == true) {
                    armRotate.setPower(0.05);
                    armBrace.setPower(0.05);
                } else {
                    if (gamepad2.right_bumper) {
                        armRotate.setPower(0.3);
                        armBrace.setPower(0.3);
                    } else if (gamepad2.left_bumper) {
                        armRotate.setPower(-0.2);
                        armBrace.setPower(-0.2);
                    } else {
                        armRotate.setPower(0);
                        armBrace.setPower(0);
                    }
                }//end arm rotation inputs


                //Arm extension controls
                //Moves up when X is pressed
                //Moves down when B is pressed
                //Otherwise set power to 0 (BRAKE)
                //Limiters are applied if the motors position is less than 350° or greater than 2900°
                //If the position is less than 350°, the arm can only extend forward
                //If the position is greater than 2900°, the arm can only retract
                if (ext >= 100 && ext < 2900) {
                    if (gamepad2.dpad_up) {
                        armExt.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext <= 100 && ext < 2900) {
                    if (gamepad2.dpad_up) {
                        armExt.setPower(1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext > 2900 && ext >= 100) {
                    if (gamepad2.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                }
                if (gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5) {
                    armExt.setPower(-1);
                    sleep(1000000);
                } //end arm extension end

                if (gamepad1.b) {
                    revBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                } else if (gamepad1.x) {
                    revBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                } else if (gamepad1.a) {
                    revBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                } else if (gamepad1.y) {
                    revBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE);
                }

                //Gripper
                if (gamepad2.a) {
                    servoLeft.setPosition(0);
                    servoRight.setPosition(1);
                } else if (gamepad2.b) {
                    servoLeft.setPosition(0.3);
                    servoRight.setPosition(0.7);
                } else if (gamepad2.x) {
                    servoLeft.setPosition(0);
                } else if (gamepad2.y) {
                    servoRight.setPosition(1);
                }

                if (gamepad2.dpad_left && gamepad1.dpad_left) {
                    servoDrone.setPosition(1);
                    sleep(1000);
                    servoDrone.setPosition(0);
                }


                //Telemetry for debugging
                telemetry.addData("Current Arm Extension", ext);
                telemetry.addData("Current Arm Rotation", armRotate.getCurrentPosition());
                telemetry.addData("Arm lift Power", armRotate.getPower());
                telemetry.addData("Back Right Wheel Power", backRightMotor.getPower());
                telemetry.addData("Back Left Wheel Power", backLeftMotor.getPower());
                telemetry.addData("Front Right Motor Power", frontRightMotor.getPower());
                telemetry.addData("Front Left Motor Power", frontLeftMotor.getPower());
                telemetry.addData("Arm Locked: ", armLocked);
                telemetry.addData("Drone Servo", servoDrone.getPosition());
                telemetry.update();
            } //end while loop
        } //end if loop
    } //end run method
}
//end class
//Nothing