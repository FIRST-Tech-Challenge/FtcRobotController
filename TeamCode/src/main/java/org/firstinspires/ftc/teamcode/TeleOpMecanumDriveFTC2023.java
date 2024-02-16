package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Tele-Op", group="Robot")

public class TeleOpMecanumDriveFTC2023 extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;

    public DcMotor  Climber = null;
    public DcMotor  climber2 = null;
    public Servo  claw = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    double position = 0;

    public DigitalChannelImpl mySwitch = null;

    //double clawOffset = 0;

    //public static final double MID_SERVO   =  0.5 ;
   // public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    //public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    //public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {
        // Define and Initialize Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "LF");
        backLeft  = hardwareMap.get(DcMotor.class, "LB");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        backRight  = hardwareMap.get(DcMotor.class, "RB");
        Climber  = hardwareMap.get(DcMotor.class, "Climber");
        climber2  = hardwareMap.get(DcMotor.class, "climber2");
        claw = hardwareMap.get(Servo.class, "Claw");
        arm1  = hardwareMap.get(Servo.class, "Arm1");
        arm2  = hardwareMap.get(Servo.class, "Arm2");
        mySwitch = hardwareMap.get(DigitalChannelImpl.class, "Touch Switch");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftArm    = hardwareMap.get(DcMotor.class, "left_arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        Climber.setDirection(DcMotor.Direction.FORWARD);
        climber2.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(Servo.Direction.REVERSE);
        arm1.scaleRange(0,1);
        arm2.setDirection(Servo.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        //rightClaw = hardwareMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void driveStraight(double pwr, int dir) {
        // dir must be -1 to 1 (to specify direction)
        frontLeft.setPower(pwr*dir);
        frontRight.setPower(pwr*dir);
        backLeft.setPower(pwr*dir);
        backRight.setPower(pwr*dir);
    }
    public void driveSide(double pwr, int dir) {
        // dir must be -1 to 1 (to specify direction)
        frontLeft.setPower(pwr*-1*dir);
        frontRight.setPower(pwr*dir);
        backLeft.setPower(pwr*dir);
        backRight.setPower(pwr*-1*dir);
    }
    public void driveTurn(double pwr, int dir) {
        double rightSideDecrementCoefficent = 0.75; // 0.5 half
        // dir must be -1 to 1 (to specify direction)
        frontLeft.setPower(pwr*-0.5*dir);
        frontRight.setPower(pwr*rightSideDecrementCoefficent*dir);
        backLeft.setPower(pwr*-0.5*dir);
        backRight.setPower(pwr*rightSideDecrementCoefficent*dir);
    }
    public void driveArm(double pos) {
        arm1.setPosition(pos);
    }
    public void start() {
        telemetry.clear();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void loop() {
        double left1y;
        double left1x;
        double right1x;
        double servoValue;
        boolean limitSwitchOne;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        left1y = gamepad1.left_stick_y;
        left1x = -gamepad1.left_stick_x;
        right1x = -gamepad1.right_stick_x;

        double baseSpeed = 0.30;

        if (gamepad1.a){
            baseSpeed = 0.15;
        }

        double denominator = Math.max(Math.abs(left1y) + Math.abs(left1x) + Math.abs(right1x), 1);
        double frontLeftPower = (left1y + left1x + right1x) / denominator;
        double backLeftPower = (left1y - left1x + right1x) / denominator;
        double frontRightPower = (left1y - left1x - right1x) / denominator;
        double backRightPower = (left1y + left1x - right1x) / denominator;
        frontLeft.setPower(frontLeftPower*baseSpeed);
        frontRight.setPower(frontRightPower*baseSpeed);
        backLeft.setPower(backLeftPower*baseSpeed);
        backRight.setPower(backRightPower*baseSpeed);
        telemetry.addData("FL",  "%.2f", frontLeftPower);
        telemetry.addData("FR",  "%.2f", frontRightPower);
        telemetry.addData("BL",  "%.2f", backLeftPower);
        telemetry.addData("BR",  "%.2f", backRightPower);

        telemetry.addData("FLspeed",  "%.2f", frontLeft.getPower());
        telemetry.addData("FRspeed",  "%.2f", frontRight.getPower());
        telemetry.addData("BLspeed",  "%.2f", backLeft.getPower());
        telemetry.addData("BRspeed",  "%.2f", backRight.getPower());

        if (mySwitch.getState()) {
            telemetry.addData("switch", mySwitch.getState());
        }

        if (gamepad2.left_stick_y > 0.1) {
            climber2.setPower(-1);
        } else if (gamepad2.left_stick_y < -0.1) {
            climber2.setPower(1);
        } else {
            climber2.setPower(0);
        }

        if (gamepad2.right_stick_y > 0.1) {
            Climber.setPower(0.25);
        } else if (gamepad2.right_stick_y < -0.1) {
            Climber.setPower(-0.25);
        } else {
            Climber.setPower(0);
        }
        // "1" position is at the closed poesition
        // furthest right is to the side of robot
        // furthest left is a small part past 1 position



        }
//

        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);

    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void stop() {
    }
}
