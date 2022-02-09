package org.firstinspires.ftc.Team19567.opmode; //The "namespace" for the project is declared here. To distinguish it from other teams' packages, Team19567 is used.

//Import necessary packages/libraries

//FTC SDK packages
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//Custom enums and classes
import org.firstinspires.ftc.Team19567.util.PRESET_STATE;
import org.firstinspires.ftc.Team19567.util.Utility_Constants;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.Team19567.util.TELEOP_STATE;


@TeleOp(name="TeleOP", group="Dababy") //Gives the TeleOp its name in the driver station menu and categorizes it as an Iterative OpMode
public class TeleOP extends OpMode {           //Declares the class TestOPIterative, which is a child of OpMode
    //Declare OpMode members

    //ElapsedTime
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime carouselTime = new ElapsedTime();

    //Hardware
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private DcMotorEx armDC = null;
    private Servo releaseServo = null;
    private Servo balanceServo = null;
    private TouchSensor limitSwitch = null;
    private DistanceSensor distanceSensor = null;
    private RevBlinkinLedDriver blinkin = null;
    private double armPos = 0;
    private double armPower = 1.0;
    private double releaseServoPos = 0.8;
    private double balanceServoPos = 0.0;
    private boolean isSlowmode = false;
    private final static double turnSens = Utility_Constants.TURN_SENSITIVITY;
    private final static double strafeSens = Utility_Constants.STRAFE_SENSITIVITY;
    private TELEOP_STATE currentState = TELEOP_STATE.DRIVER_CONTROL;
    private double acc = 1.0;

    //Objects
    private RevBlinkinLedDriver.BlinkinPattern blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
    private Mechanisms mechanisms = null;
    private PIDFCoefficients armCoefficients = new PIDFCoefficients(8,0,1,0);

    public int milliCarousel = 2000;
    public double initPower = 0.6;
    public double finalPower = 1;

    private PRESET_STATE presetState = PRESET_STATE.NO_PRESET;

    @Override
    public void init() {
        //Get the motors from the robot's configuration

        leftDCFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightDCFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftDCBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDCBack = hardwareMap.get(DcMotor.class, "rightBack");
        armDC = hardwareMap.get(DcMotorEx.class, "armDC");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        balanceServo = hardwareMap.get(Servo.class, "balanceServo");
        limitSwitch = hardwareMap.get(TouchSensor.class,"limitSwitch");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        //Set direction to be forward in case the robot's motors are oriented otherwise; can change FORWARD to REVERSE if necessary

        leftDCFront.setDirection(DcMotor.Direction.FORWARD);
        rightDCFront.setDirection(DcMotor.Direction.REVERSE);
        leftDCBack.setDirection(DcMotor.Direction.FORWARD);
        rightDCBack.setDirection(DcMotor.Direction.REVERSE);
        armDC.setDirection(DcMotor.Direction.REVERSE);
        balanceServo.setDirection(Servo.Direction.REVERSE);

        releaseServoPos = 0.8;
        balanceServoPos = balanceServo.MIN_POSITION;
        mechanisms = new Mechanisms(hardwareMap,telemetry);

        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Awaiting Start");
        telemetry.update();
    }

    @Override
    public void start() {
        armDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDC.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,armCoefficients);
        mechanisms.setModes();
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
        if(gamepad1.y) isSlowmode = !isSlowmode;
        if(isSlowmode) acc = Utility_Constants.SLOWMODE_MULT;
        else acc = 1.0;
        final double leftFrontSpeed = (strafeSens * r * Math.sin(angleDC) - turnSens*gamepad1.right_stick_x)*acc; //Using the math explained above, we can obtain the values we want to multiply each wheel by. acc is the variable which controls the overall multiplier of how fast we want to go.
        final double rightFrontSpeed = (strafeSens * r * Math.cos(angleDC) + turnSens*gamepad1.right_stick_x)*acc;
        final double leftBackSpeed = (strafeSens * r * Math.cos(angleDC) - turnSens*gamepad1.right_stick_x)*acc;
        final double rightBackSpeed = (strafeSens * r * Math.sin(angleDC) + turnSens*gamepad1.right_stick_x)*acc;

        //INTAKE
        if(gamepad1.right_trigger > 0) mechanisms.moveIntake(1);
        else if(gamepad1.right_bumper) mechanisms.moveIntake(-1);
        else if(limitSwitch.isPressed()) mechanisms.moveIntake(0.0);
        else mechanisms.moveIntake(0.1);

        //CAROUSEL
        /*if(gamepad1.dpad_right || gamepad2.dpad_right) {
            long start = System.nanoTime();
            mechanisms.rotateCarousel(0.5);
            long end = System.nanoTime();
            long elapsedtime = end - start;
            if (elapsedtime >= 10);
            mechanisms.rotateCarousel(1);
        }
        else if(gamepad1.dpad_left || gamepad2.dpad_left) {
            mechanisms.rotateCarousel(0.5);
        }
        else {
            mechanisms.rotateCarousel(0.0);
        }*/

        if(gamepad2.right_trigger > 0 || gamepad1.dpad_right || gamepad1.dpad_left)
        {
            carouselTime.reset();
            if(carouselTime.milliseconds() <= milliCarousel) mechanisms.rotateCarousel(initPower);
            else mechanisms.rotateCarousel(finalPower);
        }
        else mechanisms.rotateCarousel(0.0);

        //ARM
        if(gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) armPos = Range.clip(armPos+gamepad1.left_trigger*4,0,1000);
        else if(gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) armPos = Range.clip(armPos+gamepad2.left_trigger*4,0,1000);
        if(gamepad1.left_bumper || gamepad2.left_bumper) armPos = Range.clip(armPos-4,0,1000);
        if(gamepad1.x || gamepad2.x) {
            presetState = PRESET_STATE.ALLIANCE_FIRST;
            currentState = TELEOP_STATE.IN_A_PRESET;
        }
        else if(gamepad2.y) {
            presetState = PRESET_STATE.ALLIANCE_THIRD;
            currentState = TELEOP_STATE.IN_A_PRESET;
        }
        else if(gamepad1.a || gamepad2.a) {
            presetState = PRESET_STATE.ALLIANCE_THIRD;
            currentState = TELEOP_STATE.IN_A_PRESET;
        }
        else if(gamepad1.b || gamepad2.b) {
            presetState = PRESET_STATE.GOING_DOWN;
            currentState = TELEOP_STATE.IN_A_PRESET;
        }

        //PRESET HANDLING
        if(currentState == TELEOP_STATE.IN_A_PRESET) {
            armPower = 0.8;
            switch(presetState) {
                case ALLIANCE_FIRST: {
                    armPower = 0.8;
                    armPos = Utility_Constants.FIRST_LEVEL_POS;
                    if(armDC.getCurrentPosition() >= Utility_Constants.FIRST_LEVEL_POS-2) {
                        presetState = PRESET_STATE.NO_PRESET;
                    }
                    break;
                }
                case ALLIANCE_SECOND: {
                    armPos = Utility_Constants.SECOND_LEVEL_POS;
                    if(armDC.getCurrentPosition() >= Utility_Constants.SECOND_LEVEL_POS-2) {
                        presetState = PRESET_STATE.NO_PRESET;
                    }
                    break;
                }
                case ALLIANCE_THIRD: {
                    armPos = Utility_Constants.THIRD_LEVEL_POS;
                    if(armDC.getCurrentPosition() >= Utility_Constants.THIRD_LEVEL_POS-5) {
                        presetState = PRESET_STATE.NO_PRESET;
                    }
                    break;
                }
                case GOING_DOWN: {
                    armPower = 0.6;
                    armPos = 0;
                    releaseServoPos = 0.8;
                    if(armDC.getCurrentPosition() <= 5) {
                        presetState = PRESET_STATE.NO_PRESET;
                    }
                }
                default: {
                    break;
                }
            }
        }

        //SERVOS
        if(gamepad1.dpad_down || gamepad2.dpad_down) releaseServoPos = Range.clip(releaseServoPos-0.01,releaseServo.MIN_POSITION,0.9);
        else if(gamepad1.dpad_up || gamepad2.dpad_up) releaseServoPos = Range.clip(releaseServoPos+0.01,releaseServo.MIN_POSITION,0.9);
        if(gamepad2.right_bumper) releaseServoPos = 0.3;
        balanceServoPos = Range.clip((armDC.getCurrentPosition()-100)/1050.1,balanceServo.MIN_POSITION,balanceServo.MAX_POSITION);
        if((runtime.milliseconds() >= 85000 && runtime.milliseconds() <= 90000) || (runtime.milliseconds() >= 115000)) blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;
        else if(presetState != PRESET_STATE.NO_PRESET) blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE;
        /* else if(distanceSensor.getDistance(DistanceUnit.MM) <= 80) {
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
            telemetry.addData("Distance Sensor","Freight Detected");
        } */
        else blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
        blinkin.setPattern(blinkinPattern);

//MOTOR SET POWER
        leftDCFront.setPower(leftFrontSpeed); //Set all the motors to their corresponding powers/speeds
        rightDCFront.setPower(rightFrontSpeed);
        leftDCBack.setPower(leftBackSpeed);
        rightDCBack.setPower(rightBackSpeed);
        armDC.setTargetPosition((int) armPos);
        armDC.setPower(armPower);
        releaseServo.setPosition(releaseServoPos);
        balanceServo.setPosition(balanceServoPos);
//TELEMETRY
        telemetry.addData("Status", "Looping"); //Add telemetry to show that the program is currently in the loop function
        telemetry.addData("Runtime", runtime.toString() + " Milliseconds"); //Display the runtime
        telemetry.addData("DCMotors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack(%.2f), armDC(%.2f)",
                leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed, armPos); //In (%.2f), the % means that special modifier is to follow, that modifier being .2f. In .2f, the .2 means to round to to digits after the decimal point, and the f means that the value to be rounded is a float.
        //(%.2f) is used here so that the displayed motor speeds aren't excessively long and don't cldfasdfasdtter(andy's one contribution) the screen.
        telemetry.addData("Servos","releaseServoPos(%.2f), balanceServoPos(%.2f)",releaseServoPos, balanceServoPos);
        telemetry.addData("Carousel Time(%.2f)",carouselTime.milliseconds());
        telemetry.update(); //Updates the telemetry
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.addData("Runtime","Total runtime was " + runtime.toString() + " Milliseconds");
        telemetry.update();
    }
}