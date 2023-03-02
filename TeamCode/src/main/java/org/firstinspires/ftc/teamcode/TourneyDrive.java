package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2022 Steering Tourney Drive", group="Linear Opmode")

public class TourneyDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
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

    LiftState liftState = LiftState.LIFT_START;
    boolean autoLift = false;
    double targetPos = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        liftMotor  = hardwareMap.get(DcMotor.class, "lift_motor");
        Servo servoGrabber1 = hardwareMap.get(Servo.class, "servo_grabber_one");
        Servo servoGrabber2 = hardwareMap.get(Servo.class, "servo_grabber_two");

        CRServo coneFlipper = hardwareMap.get(CRServo.class, "cone_flipper");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoGrabber1.setPosition(position);
        servoGrabber2.setPosition(position2);

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("FL Encoder:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("BL Encoder:", leftBackDrive.getCurrentPosition());
            telemetry.addData("FR Encoder:", rightFrontDrive.getCurrentPosition());
            telemetry.addData("BR Encoder:", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

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
                        liftState = LiftState.LIFT_RAISE;
                    } else if (liftLow) {
                        autoLift = true;
                        moveLift(1, (173 * 15) + MIN_LIFT_POS);
                        liftState = LiftState.LIFT_RAISE;
                    } else if (liftMedium) {
                        autoLift = true;
                        moveLift(1, (173 * 25) + MIN_LIFT_POS);
                        liftState = LiftState.LIFT_RAISE;
                    } else if (liftHigh) {
                        autoLift = true;
                        moveLift(1, MAX_LIFT_POS);
                        liftState = LiftState.LIFT_RAISE;
                    }
                    break;
                case LIFT_RAISE:
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (liftMotor.getCurrentPosition() <= targetPos + 173 && liftMotor.getCurrentPosition() >= targetPos - 173) {
                        moveLift(.25, targetPos);
                        liftState = LiftState.LIFT_CORRECT;
                    }
                    break;
                case LIFT_CORRECT:
                    if (liftMotor.getCurrentPosition() <= targetPos + 17.3 && liftMotor.getCurrentPosition() >= targetPos - 17.3) {
                        liftMotor.setPower(0);
                        liftState = LiftState.LIFT_START;
                        autoLift = false;
                    }
                    break;
                default:
                    liftState = LiftState.LIFT_START;
                    break;
            }

            if (stopAutoLift && liftState != LiftState.LIFT_START) {
                liftState = LiftState.LIFT_START;
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

            if(adjustForward){
                lAdjust = .25;
                lbAdjust = .25;
                rAdjust = .25;
                rbAdjust = .25;
            }else if(adjustBackward){
                lAdjust = -.25;
                lbAdjust = -.25;
                rAdjust = -.25;
                rbAdjust = -.25;
            }else if(adjustRightStrafe){
                lAdjust = .25;
                lbAdjust = -.25;
                rAdjust = -.25;
                rbAdjust = .25;
            }else if(adjustLeftStrafe){
                lAdjust = -.25;
                lbAdjust = .25;
                rAdjust = .25;
                rbAdjust = -.25;
            }else if(adjustLeftTurn){
                lAdjust = -.25;
                lbAdjust = -.25;
                rAdjust = .25;
                rbAdjust = .25;
            }else if(adjustRightTurn){
                lAdjust = .25;
                lbAdjust = .25;
                rAdjust = -.25;
                rbAdjust = -.25;
            }else{
                lAdjust = 0;
                lbAdjust = 0;
                rAdjust = 0;
                rbAdjust = 0;
            }

            if(raiseMaxHeight && runtime.seconds() >= lastAdjusted + .5){
                MAX_LIFT_POS += 50;
                MIN_LIFT_POS += 50;
                lastAdjusted = runtime.seconds();
            }

            if(lowerMaxHeight && runtime.seconds() >= lastAdjusted + .5){
                MAX_LIFT_POS -= 50;
                MIN_LIFT_POS -= 50;
                lastAdjusted = runtime.seconds();
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw + lAdjust;
            double rightFrontPower = axial - lateral - yaw + rAdjust;
            double leftBackPower   = axial - lateral + yaw + lbAdjust;
            double rightBackPower  = axial + lateral - yaw + rbAdjust;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Servo power: ", coneFlipper.getPower());
            telemetry.update();
        }
    }

    /**
     * Move lift to a set height in ticks
     * @param power Percent of how fast to move
     * @param height Height to go to in ticks
     */
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