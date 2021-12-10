package org.firstinspires.ftc.Team19567; //The "namespace" for the project is declared here. To distinguish it from other teams' packages, Team19567 is used.

//Import necessary packages/libraries

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOP", group="Iterative Opmode") //Gives the TeleOp its name in the driver station menu and categorizes it as a TeleOp (Iterative OpMode)
public class TeleOPYUien extends OpMode {
    //Declares the class TestOPIterative, which is a child of OpMode
    //Declare OpMode members
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private DcMotor carouselDC = null;
    private DcMotor intakeDC = null;
    private DcMotor releaseDC = null;
    private Servo releaseServo = null;
    private Servo intakeServo = null;
    private double carouselPower = 0.0;
    private double releaseDcPower = 1.0;
    private double releaseDcPos = 0.0;
    private double releaseServoPos = 0.0;
    private double intakeServoPos = 0.0;
    private boolean isSlowmode = false;
    private double acc = 1.0;
    private int presetDelay = 0;
    private TELEOPLOCATIONS location = null;

    public enum TELEOPLOCATIONS {
        SHARED_HUB,
        ALLIANCE_FIRST,
        ALLIANCE_SECOND,
        ALLIANCE_THIRD
    }

    @Override
    public void init() {
        //Get the motors from the robot's configuration

        leftDCFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightDCFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftDCBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDCBack = hardwareMap.get(DcMotor.class, "rightBack");
        carouselDC = hardwareMap.get(DcMotor.class, "carouselDC");
        intakeDC = hardwareMap.get(DcMotor.class, "intakeDC");
        releaseDC = hardwareMap.get(DcMotor.class, "releaseDC");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        //Set direction to be forward in case the robot's motors are oriented otherwise; can change FORWARD to REVERSE if necessary

        leftDCFront.setDirection(DcMotor.Direction.FORWARD);
        rightDCFront.setDirection(DcMotor.Direction.REVERSE);
        leftDCBack.setDirection(DcMotor.Direction.FORWARD);
        rightDCBack.setDirection(DcMotor.Direction.REVERSE);
        intakeDC.setDirection(DcMotor.Direction.REVERSE);
        releaseDC.setDirection(DcMotor.Direction.FORWARD);
//put in new motor and servo for release
        releaseServoPos = releaseServo.MAX_POSITION;
        intakeServoPos = intakeServo.MIN_POSITION;

        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Awaiting Start");
        telemetry.update();
    }

    @Override
    public void start() {
        releaseDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        releaseDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Started");
        telemetry.update();
        runtime.reset(); //Reset runtime
    }

    @Override
    public void loop() {
        /* Mecanum drive applies forces at a 45 degree angle to its wheels, instead of directly (like most non-holonomic chassis);
        this means that its axis of movement is not the tradition al x-y plane, but rather said plane rotated 45 degrees. This also means that the wheels
        of a mecanum chassis should be as close to a square as possible. That way, the rotated plane's axes push straight through the wheels for
        maximum efficiency. */

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //Gets the amount that we want to translate
        double angleDC = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; /* This gets the angle theta we desire to turn at. It is subtracted
        by PI/4 radians, which equals 45 degrees. One can also add 45 degrees. It doesn't matter so long as the axes of the robot are rotated 45 degrees. */

        /* The axes we want to move on are the hypotenuse of the right triangle with non-right angle angleDC. The size of this hypotenuse is determined by the
        value of r, which we computed earlier. Thus, the amount that we want to move in the standard x-y plane are the legs of the right triangle. To determine
        which wheels drive th x leg and which drive the y leg (base), we must take a look at the configuration of the wheels. In most robots, which have the wheels
        pointed outwards in an X shape (maximum efficiency), we see that the front left and rear right wheels point in the y direction, while the rear left
        and front right wheels point in the x direction. Thus, we use cosine for the first group and sine for the second group. */
//DRIVETRAIN
        if (gamepad1.y) isSlowmode = !isSlowmode;
        if (isSlowmode) acc = 0.3;
        else acc = 1.0;
        final double leftFrontSpeed = (r * Math.sin(angleDC) - gamepad1.right_stick_x) * acc; //Using the math explained above, we can obtain the values we want to multiply each wheel by. acc is the variable which controls the overall multiplier of how fast we want to go.
        final double rightFrontSpeed = (r * Math.cos(angleDC) + gamepad1.right_stick_x) * acc;
        final double leftBackSpeed = (r * Math.cos(angleDC) - gamepad1.right_stick_x) * acc;
        final double rightBackSpeed = (r * Math.sin(angleDC) + gamepad1.right_stick_x) * acc;
        //INTAKE
        double intakePower = 0.0;
        if (gamepad2.right_trigger > 0) intakePower = gamepad2.right_trigger;
        if (gamepad1.right_trigger > 0) intakePower = gamepad1.right_trigger;
        if (gamepad1.right_bumper || gamepad2.right_bumper) intakePower = -1.0;
        //CAROUSEL
        if (gamepad1.dpad_left || gamepad2.dpad_left) carouselPower = -0.5;
        else if (gamepad1.dpad_right || gamepad2.dpad_right) carouselPower = 0.5;
        else carouselPower = 0.0;
        //LINEAR SLIDE
        if (gamepad1.left_trigger > 0) releaseDcPos += gamepad1.left_trigger * 5;
        else if (gamepad2.left_trigger > 0) releaseDcPos += gamepad2.left_trigger * 5;
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            releaseDcPos = Range.clip(releaseDcPos - 10, 0.0, releaseDcPos - 5);
        }

//SERVOS
        if (gamepad1.dpad_down || gamepad2.dpad_down)
            releaseServoPos = Range.clip(releaseServoPos + 0.006, releaseServo.MIN_POSITION, releaseServo.MAX_POSITION);
        else if (gamepad1.dpad_up || gamepad2.dpad_up)
            releaseServoPos = Range.clip(releaseServoPos - 0.006, releaseServo.MIN_POSITION, releaseServo.MAX_POSITION);

        if (gamepad2.b) {
            releaseDcPos = 0;
            releaseServoPos = releaseServo.MAX_POSITION;
        }

        if (gamepad2.y) location = TELEOPLOCATIONS.ALLIANCE_SECOND;
        if (gamepad2.x) location = TELEOPLOCATIONS.SHARED_HUB;
        if (gamepad2.y) location = TELEOPLOCATIONS.ALLIANCE_THIRD;

        switch (location) {
            case SHARED_HUB: {
                presetDelay++;
                releaseDcPos = 1000;
                if (presetDelay >= 10) {
                    releaseServoPos = releaseServo.MIN_POSITION;
                    presetDelay = 0;
                }
                switch (location) {
                    case ALLIANCE_SECOND: {
                        presetDelay++;
                        releaseDcPos = 1000;
                        if (presetDelay >= 40) {
                            releaseServoPos = releaseServo.MIN_POSITION;
                            presetDelay = 0;
                        }
                        switch (location) {
                            case ALLIANCE_THIRD: {
                                presetDelay++;
                                releaseDcPos = 1000;
                                if (presetDelay >= 100) {
                                    releaseServoPos = releaseServo.MIN_POSITION;
                                    presetDelay = 0;
                                }
                            }
                            default: {
                                break;
                            }
                        }

//MOTOR SET POWER
                        leftDCFront.setPower(leftFrontSpeed); //Set all the motors to their corresponding powers/speeds
                        rightDCFront.setPower(rightFrontSpeed);
                        leftDCBack.setPower(leftBackSpeed);
                        rightDCBack.setPower(rightBackSpeed);
                        carouselDC.setPower(carouselPower);
                        intakeDC.setPower(intakePower);
                        releaseDC.setTargetPosition((int) releaseDcPos);
                        releaseDC.setPower(releaseDcPower);
                        releaseDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        releaseServo.setPosition(releaseServoPos);
                        intakeServo.setPosition(intakeServoPos);
//TELEMETRY
                        telemetry.addData("Status", "Looping"); //Add telemetry to show that the program is currently in the loop function
                        telemetry.addData("Runtime", runtime.toString() + " Milliseconds"); //Display the runtime
                        telemetry.addData("DCMotors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack(%.2f), carouselDC(%.2f), intakeDC(%.2f), releaseDC(%.2f), releaseServo(%.2f)",
                                leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed, carouselPower, intakePower, releaseDcPos, releaseServoPos); //In (%.2f), the % means that special modifier is to follow, that modifier being .2f. In .2f, the .2 means to round to to digits after the decimal point, and the f means that the value to be rounded is a float.
                        //(%.2f) is used here so that the displayed motor speeds aren't excessively long and don't cldfasdfasdtter(andy's one contribution) the screen.
                        telemetry.update(); //Updates the telemetry
                    }
                }
            }
        }

    }
}