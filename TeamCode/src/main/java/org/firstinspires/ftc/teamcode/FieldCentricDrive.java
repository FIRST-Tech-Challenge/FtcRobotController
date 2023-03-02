package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor liftMotor = null;

    static final double MAX_POS     =    .72;
    static final double MAX_POS2    =    .28;
    static final double MIN_POS     =     1;
    static final double MIN_POS2    =     0;

    double MIN_LIFT_POS = 0;
    double MAX_LIFT_POS = 173 * 34.5;

    double position     =   1;
    double position2    =   0;

    double lAdjust  =   0;
    double lbAdjust =   0;
    double rAdjust  =   0;
    double rbAdjust =   0;

    double lastAdjusted = runtime.seconds();
    double lastSwitched = runtime.seconds();

    public enum LiftState{
        LIFT_START,
        LIFT_RAISE,
        LIFT_CORRECT,
    }

    TourneyDrive.LiftState liftState = TourneyDrive.LiftState.LIFT_START;
    boolean autoLift = false;
    double targetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("right_back_drive");

        liftMotor  = hardwareMap.get(DcMotor.class, "lift_motor");
        Servo servoGrabber1 = hardwareMap.get(Servo.class, "servo_grabber_one");
        Servo servoGrabber2 = hardwareMap.get(Servo.class, "servo_grabber_two");

        CRServo coneFlipper = hardwareMap.get(CRServo.class, "cone_flipper");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoGrabber1.setPosition(position);
        servoGrabber2.setPosition(position2);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            boolean adjustLeftStrafe = gamepad1.dpad_left;
            boolean adjustRightStrafe = gamepad1.dpad_right;
            boolean adjustBackward = gamepad1.dpad_down;
            boolean adjustForward = gamepad1.dpad_up;
            boolean adjustLeftTurn = gamepad1.left_bumper;
            boolean adjustRightTurn = gamepad1.right_bumper;

            float coneFlipUp = gamepad1.right_trigger;
            float coneFlipDown = gamepad1.left_trigger;

            double liftFast = -gamepad2.left_stick_y;
            double liftSlow = -gamepad2.right_stick_y;

            boolean raiseMaxHeight = gamepad2.y;
            boolean lowerMaxHeight = gamepad2.x;

            boolean liftDown = gamepad2.dpad_down;
            boolean liftLow = gamepad2.dpad_left;
            boolean liftMedium = gamepad2.dpad_right;
            boolean liftHigh = gamepad2.dpad_up;

            boolean stopAutoLift = gamepad2.a;

            boolean grabberOpen = gamepad2.left_bumper;
            boolean grabberClose = gamepad2.right_bumper;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if(!autoLift) {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (liftFast > .05 || liftFast < -.05 || liftSlow > .05 || liftSlow < -.05 && (!(liftMotor.getCurrentPosition() > MAX_LIFT_POS) || !(liftMotor.getCurrentPosition() < MIN_LIFT_POS))) {
                    if (liftFast > .05 || liftFast < -.05) {
                        if (liftFast > .05 && liftMotor.getCurrentPosition() < MAX_LIFT_POS) {
                            liftMotor.setPower(1);
                        } else if (liftFast < -.05 && liftMotor.getCurrentPosition() > MIN_LIFT_POS) {
                            liftMotor.setPower(-1);
                        } else {
                            liftMotor.setPower(0);
                        }
                    } else {
                        if (liftSlow > .05 && liftMotor.getCurrentPosition() < MAX_LIFT_POS) {
                            liftMotor.setPower(.5);
                        } else if (liftSlow < -.05 && liftMotor.getCurrentPosition() > MIN_LIFT_POS) {
                            liftMotor.setPower(-.4);
                        } else {
                            liftMotor.setPower(0);
                        }
                    }
                } else {
                    liftMotor.setPower(0);
                }
            }

            switch (liftState) {
                case LIFT_START:
                    if (liftDown) {
                        autoLift = true;
                        moveLift(1, MIN_LIFT_POS);
                        liftState = TourneyDrive.LiftState.LIFT_RAISE;
                    } else if (liftLow) {
                        autoLift = true;
                        moveLift(1, (173 * 15) + MIN_LIFT_POS);
                        liftState = TourneyDrive.LiftState.LIFT_RAISE;
                    } else if (liftMedium) {
                        autoLift = true;
                        moveLift(1, (173 * 25) + MIN_LIFT_POS);
                        liftState = TourneyDrive.LiftState.LIFT_RAISE;
                    } else if (liftHigh) {
                        autoLift = true;
                        moveLift(1, MAX_LIFT_POS);
                        liftState = TourneyDrive.LiftState.LIFT_RAISE;
                    }
                    break;
                case LIFT_RAISE:
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (liftMotor.getCurrentPosition() <= targetPos + 173 && liftMotor.getCurrentPosition() >= targetPos - 173) {
                        moveLift(.25, targetPos);
                        liftState = TourneyDrive.LiftState.LIFT_CORRECT;
                    }
                    break;
                case LIFT_CORRECT:
                    if (liftMotor.getCurrentPosition() <= targetPos + 17.3 && liftMotor.getCurrentPosition() >= targetPos - 17.3) {
                        liftMotor.setPower(0);
                        liftState = TourneyDrive.LiftState.LIFT_START;
                        autoLift = false;
                    }
                    break;
                default:
                    liftState = TourneyDrive.LiftState.LIFT_START;
                    break;
            }

            if (stopAutoLift && liftState != TourneyDrive.LiftState.LIFT_START) {
                liftState = TourneyDrive.LiftState.LIFT_START;
                autoLift = false;
            }

            if(grabberClose){
                servoGrabber1.setPosition(MAX_POS);
                servoGrabber2.setPosition(MAX_POS2);
            }else if(grabberOpen){
                servoGrabber1.setPosition(MIN_POS);
                servoGrabber2.setPosition(MIN_POS2);
            }

            if(coneFlipUp > .01){
                coneFlipper.setPower(-.75);
            }else if(coneFlipDown > .01){
                coneFlipper.setPower(.75);
            }else{
                coneFlipper.setPower(0);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bots rotation
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

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }

    public void moveLift(double power, double height){
        if(height > liftMotor.getCurrentPosition()){
            liftMotor.setPower(power);
        }else{
            liftMotor.setPower(-power);
        }
        liftMotor.setTargetPosition((int) (height));
        targetPos = height;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}