// Co-Authored by Shayne Thompson
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Energize V2 TeleOp", group="Linear Opmode")
//@Disabled
public class EnergizeV2TeleOp extends LinearOpMode {

    // Declares OpMode members for needed motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private ElapsedTime waitTimeToggle = new ElapsedTime();
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private  DigitalChannel limitSwitch = null;

    double liftPower = 0.0;
    double INCREMENT = 0.01;
    double NEGATIVE_INCREMENT = -0.2;
    double MAX_SPEED = 0.7;
    double MIN_SPEED = 0.0;

    double MAX_DRIVE_POWER = 0.75;

    double PRECISION_MODE_THROTTLE = 2;
    //private DcMotor lift = null;
    Servo rightServo;
    Servo leftServo;
    double servoPosition = 0.0;

    private final double MAX_POWER = 1.0;
    private final double MAX_HEIGHT = 90;
    private final double LIFT_POWER_INCREMENT = 0.05;
    // when moving to a target height with the lift, this is how much flexibility the lift has
    // in reaching the target height [cm]
    private final double WIGGLE_ROOM = 1.5;

    // target heights [cm]
    private final double LOW_HEIGHT = 36.0;
    private final double MED_HEIGHT = 60.0;
    private final double HIGH_HEIGHT = 89.0;

    private DistanceSensor sensorRange;
    private DigitalChannel sensorTouch;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    boolean toggle = false;

    @Override
    public void runOpMode() {

        // Corresponds driving motor names to motor variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        redLED = hardwareMap.get(DigitalChannel.class, "redLED");
        greenLED = hardwareMap.get(DigitalChannel.class, "greenLED");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        // Corresponds driving servo names to servo variables and set beginning position.
        rightServo = hardwareMap.get(Servo.class,"rightservo");
        rightServo.setPosition(servoPosition);
        leftServo = hardwareMap.get(Servo.class,"leftservo");
        leftServo.setPosition(1.0);

        // Corresponds lift motor names to motor variables.
        liftMotor1  = hardwareMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        sensorTouch = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);

        // Initializes motor directions.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        //MoveLift(0.0);

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        // Waits for driver to press play.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Runs until driver presses stop.
        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = 0;
            double rightFrontPower = 0;
            double leftBackPower = 0;
            double rightBackPower = 0;

            if (Math.abs(axial) > 0.05 || Math.abs(lateral) > 0.05 || Math.abs(yaw) > 0.05) {
                leftFrontPower =  axial + lateral + yaw;
                rightFrontPower = axial - lateral - yaw;
                leftBackPower = axial - lateral + yaw;
                rightBackPower = axial + lateral - yaw;
            }

            // Normalizes the values so no wheel power exceeds 100%.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (leftFrontPower > MAX_DRIVE_POWER) {
                leftFrontPower = MAX_DRIVE_POWER;
            }
            if (leftBackPower > MAX_DRIVE_POWER) {
                leftBackPower = MAX_DRIVE_POWER;
            }
            if (rightFrontPower > MAX_DRIVE_POWER) {
                rightFrontPower = MAX_DRIVE_POWER;
            }
            if (rightBackPower > MAX_DRIVE_POWER) {
                rightBackPower = MAX_DRIVE_POWER;
            }
            if (leftFrontPower < - MAX_DRIVE_POWER) {
                leftFrontPower = - MAX_DRIVE_POWER;
            }
            if (leftBackPower < - MAX_DRIVE_POWER) {
                leftBackPower = - MAX_DRIVE_POWER;
            }
            if (rightFrontPower < - MAX_DRIVE_POWER) {
                rightFrontPower = - MAX_DRIVE_POWER;
            }
            if (rightBackPower < - MAX_DRIVE_POWER) {
                rightBackPower = - MAX_DRIVE_POWER;
            }

            if (gamepad1.x && waitTimeToggle.time() > .75) {
                toggle = !toggle;
                if(toggle) {
                    redLED.setState(false);
                    greenLED.setState(true);

                }

                else{

                    redLED.setState(true);
                    greenLED.setState(false);

                }
                waitTimeToggle.reset();
            }
            if (toggle) {
                leftFrontDrive.setPower(leftFrontPower / PRECISION_MODE_THROTTLE);
                rightFrontDrive.setPower(rightFrontPower / PRECISION_MODE_THROTTLE);
                leftBackDrive.setPower(leftBackPower / PRECISION_MODE_THROTTLE);
                rightBackDrive.setPower(rightBackPower / PRECISION_MODE_THROTTLE);
            } else {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

            }
            // Opens claw.
            if (gamepad2.right_bumper) {
                rightServo.setPosition(1.0);
                leftServo.setPosition(0.0);
            }

            // Closes claw.
            if (gamepad2.left_bumper) {
                rightServo.setPosition(0.0);
                leftServo.setPosition(1.0);
            }

//            if (gamepad2.y)
//            {
//                MoveLift(HIGH_HEIGHT);
//            }
//            else if (gamepad2.x)
//            {
//                MoveLift(MED_HEIGHT);
//            }
//            else if (gamepad2.b)
//            {
//                MoveLift(LOW_HEIGHT);
//            }
//            else if (gamepad2.a){
//                MoveLift(-20);
//            }
//            else if(gamepad2.dpad_up){
//                MoveLift(Increment(10));
//            }
//            else if(gamepad2.dpad_down){
//                MoveLift(Increment(-10));
//            }

            if(!sensorTouch.getState() && -gamepad2.left_stick_y < 0.0) {
                dualLift(0.0);
            }
            else if (!limitSwitch.getState() && -gamepad2.left_stick_y > 0.0) {
                dualLift(0.0);
            }
            else {
                dualLift(-gamepad2.left_stick_y);
            }

            // Shows the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift Motors", "%5.2f", liftPower);
            telemetry.addData("Precision Mode",String.valueOf(toggle));
            telemetry.addData("Limit Switch", String.valueOf(limitSwitch.getState()));
            telemetry.update();
        }

    }

    public void MoveLift(double targetHeight) {

        double liftPower = 0.0;
        while (sensorRange.getDistance(DistanceUnit.CM) < targetHeight - WIGGLE_ROOM)
        {
            liftPower = RampUpLiftPower(liftPower);
            dualLift(liftPower);
        }
        liftPower = 0.0;
        dualLift(liftPower);
        while (sensorRange.getDistance(DistanceUnit.CM) > targetHeight + WIGGLE_ROOM)
        {
            if(!sensorTouch.getState()){
                dualLift(0.0);
                break;
            }
            else  {
                liftPower = RampDownLiftPower(liftPower);
                dualLift(liftPower);
            }
        }
        liftPower = 0.0;
        dualLift(liftPower);
    }

    public double RampUpLiftPower(double liftPower) {

        if (liftPower < MAX_POWER)
        {
            return liftPower + LIFT_POWER_INCREMENT;
        }
        return liftPower;
    }

    public double RampDownLiftPower(double liftPower) {

        if (liftPower > - MAX_POWER)
        {
            return liftPower - LIFT_POWER_INCREMENT;
        }
        return liftPower;
    }

    public void dualLift(double power) {

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public double Increment(double Incremented_height){

        if(Incremented_height + sensorRange.getDistance(DistanceUnit.CM) <= MAX_HEIGHT) {
            return sensorRange.getDistance(DistanceUnit.CM) + Incremented_height;
        }
        return sensorRange.getDistance(DistanceUnit.CM);

    }
}
