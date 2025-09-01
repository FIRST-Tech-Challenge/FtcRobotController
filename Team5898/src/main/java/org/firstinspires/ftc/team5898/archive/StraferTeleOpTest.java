package org.firstinspires.ftc.team5898.archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * This particular OpMode executes a POV Teleop for a mecanum robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back and strafes left & right.
 * The Right stick rotates the robot left and right.
 *
 */

@Disabled
@TeleOp(name="Strafer Tele Op - Test Code", group="Starter Code")
public class StraferTeleOpTest extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, motorArmTilt, motorBeltSlide;
    private Servo servoWrist;
    private CRServo servoClaw;

    private IMU imu;
    private final double CLAW_OPEN = 0.4;
    private final double CLAW_CLOSE = 0.27;
    final int TILT_HIGH = 2100;
    final int TILT_LOW = 200;
    final int BELT_IN = 0;
    final int BELT_OUT = -2900;
    final int SLIDE_HIGH_BASKET = 2000;
    final int SLIDE_RETURN = 1100;
    private double wristPos;

    public CRServo clawPos;

    private boolean slideLimit;


    final double CLAW_DUMP_TIME = 2.0;
    final double CLAW_RETURN_TIME = 1.5;

    // An Enum is used to represent arm states.
    // (This is one thing enums are designed to do)
    public enum ArmState {
        ARM_START,
        ARM_TILT,
        ARM_EXTEND,
        FLIP_OUT,
        SPIT_OUT,
        INTAKE_IN,
        INTAKE_OUT,
        ARM_IN,
        ARM_TILT_IN
    };

    ArmState armState = ArmState.ARM_START;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        // These are the extra moving parts

        motorArmTilt = hardwareMap.dcMotor.get("Arm");
        motorBeltSlide = hardwareMap.dcMotor.get("Belt");
        servoClaw = hardwareMap.crservo.get("Claw");

        servoWrist = hardwareMap.servo.get("Wrist");


        slideLimit = true;

        // Reverse the left side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        motorBeltSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBeltSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBeltSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmTilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        motorArmTilt.setTargetPosition(TILT_LOW);
        motorArmTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristPos = 1.0;
        servoWrist.setPosition(wristPos);

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        motorArmTilt.setPower(0.5);
        servoWrist.setPosition(wristPos);
        telemetry.addData("State: ", ""+armState);
        telemetry.addData("Slide Limited? ", slideLimit);
        telemetry.addData("Elapsed Time", runtime.seconds());
        telemetry.addData("Tilt encoder: ", motorArmTilt.getCurrentPosition());
        telemetry.addData("Slide encoder: ", motorBeltSlide.getCurrentPosition());


        switch (armState) {
            case ARM_START:
                // wait for input
                motorBeltSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad2.x) {
                    motorArmTilt.setPower(1);
                    motorArmTilt.setTargetPosition(TILT_HIGH);
                    motorArmTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wristPos = .4;
                    armState = ArmState.ARM_TILT;
                }
                else {
                    //all manual control
                    int slidePos = motorBeltSlide.getCurrentPosition();
                    int tiltPos = motorArmTilt.getCurrentPosition();
                    telemetry.addData("Current slide position", slidePos);
                    telemetry.addData("Current tilt position", tiltPos);


                    // manual slide control
                    if (slideLimit)
                    {
                        if (gamepad2.dpad_up && slidePos >= -1450)
                        {
                            motorBeltSlide.setPower(-.5);
                        }
                        else if (gamepad2.dpad_down && slidePos < -10)
                        {
                            motorBeltSlide.setPower(.5);
                        }
                        else
                        {
                            motorBeltSlide.setPower(0);
                        }
                        // manual tilt control
                        if (gamepad2.dpad_left)
                        {
                            motorArmTilt.setTargetPosition(motorArmTilt.getCurrentPosition()+50);
                        }
                        else if (gamepad2.dpad_right && tiltPos > 10)
                        {
                            motorArmTilt.setTargetPosition(motorArmTilt.getCurrentPosition()-50);
                        }
                    }
                    else{
                        if (gamepad2.dpad_up)
                        {
                            motorBeltSlide.setPower(-.5);
                        }
                        else if (gamepad2.dpad_down && slidePos < -10)
                        {
                            motorBeltSlide.setPower(.5);
                        }
                        else
                        {
                            motorBeltSlide.setPower(0);
                        }
                        // manual tilt control
                        if (gamepad2.dpad_left && tiltPos <= 2000)
                        {
                            motorArmTilt.setTargetPosition(motorArmTilt.getCurrentPosition()+50);
                        }
                        else if (gamepad2.dpad_right)
                        {
                            motorArmTilt.setTargetPosition(motorArmTilt.getCurrentPosition()-50);
                        }
                    }

                    if (gamepad2.right_trigger > 0.3)
                    {
                        motorBeltSlide.setPower(.5);
                    }

                    if (gamepad2.back) {
                        motorBeltSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motorBeltSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
                    }

                    // wrist control
                    telemetry.addData("Wrist Current Position: ", servoWrist.getPosition());
                    telemetry.addData("WristPOS: ", wristPos);
                    if (gamepad2.a && wristPos > 0.01)
                    {
                        wristPos -= .01;
                    }
                    else if (gamepad2.y && wristPos < .99)
                    {
                        wristPos += .01;
                    }


                    // manual claw control
                    if (gamepad1.left_bumper)
                    {
                        servoClaw.setPower(.5);
                    }
                    else if (gamepad1.right_bumper)
                    {
                        servoClaw.setPower(-.5);
                    }
                    else
                        servoClaw.setPower(0);


                }
                break;
            case ARM_TILT:
                if (motorArmTilt.getCurrentPosition()> TILT_HIGH - 50) {
                    motorBeltSlide.setPower(-1);
                    motorBeltSlide.setTargetPosition(-3000);
                    motorBeltSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armState = ArmState.ARM_EXTEND;
                }
                break;
            case ARM_EXTEND:
                if (motorBeltSlide.getCurrentPosition() < -2800) {
                    wristPos = .9;
                    armState = ArmState.FLIP_OUT;
                    runtime.reset();
                    motorBeltSlide.setPower(1);
                }
                break;
            case FLIP_OUT:
                if (runtime.seconds()>=1) {
                    servoClaw.setPower(1);
                    armState = ArmState.SPIT_OUT;
                    runtime.reset();
                }
                break;
            case SPIT_OUT:
                if (runtime.seconds() >= 1) {
                    servoClaw.setPower(0);
                    wristPos = .7;
                    armState = ArmState.INTAKE_IN;
                    runtime.reset();
                }
                break;
            case INTAKE_IN:
                if (runtime.seconds() >= 2) {
                    armState = ArmState.ARM_IN;
                    runtime.reset();
                    motorBeltSlide.setTargetPosition(BELT_IN);
                }
                break;
            case ARM_IN:
                if (motorBeltSlide.getCurrentPosition() >= -10){
                    armState = ArmState.ARM_TILT_IN;
                    motorArmTilt.setTargetPosition(TILT_LOW);
                    slideLimit = true;
                    armState = ArmState.ARM_START;
                }
                break;

            default:
                // should never be reached, as armState should never be null
                armState = ArmState.ARM_START;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad2.guide && armState != ArmState.ARM_START) {
            armState = ArmState.ARM_START;
        }


        // Drive Code
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers or options on PS4-style controllers.
        if (gamepad1.guide) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        double leftHangPower = (rotY + rotX - rx) / denominator;
        double rightHangPower = (rotY + rotX - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);




        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
