package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Energize V1 TeleOp", group="Linear Opmode")
//@Disabled
public class EnergizeV3TeleOp extends LinearOpMode {

    // Declares OpMode members for needed motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    double liftPower = 0.0;
    double INCREMENT = 0.01;
    double NEGATIVE_INCREMENT = -0.2;
    double MAX_SPEED = 0.7;
    double MIN_SPEED = 0.0;
    //private DcMotor lift = null;
    Servo rightServo;
    Servo leftServo;
    double servoPosition = 0.0;

    private DistanceSensor sensorRange;
    private DigitalChannel sensorTouch;

    private final double MAX_POWER = 0.75;
    private final double MAX_HEIGHT = 90;
    private final double LIFT_POWER_INCREMENT = 0.05;
    // when moving to a target height with the lift, this is how much flexibility the lift has
    // in reaching the target height [cm]
    private final double WIGGLE_ROOM = 1.5;

    // target heights [cm]
    private final double LOW_HEIGHT = 36;
    private final double MED_HEIGHT = 60;
    private final double HIGH_HEIGHT = 86.4;

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

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        sensorTouch = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);


        // Corresponds driving servo names to servo variables and set beginning position.
        rightServo = hardwareMap.get(Servo.class,"rightservo");
        rightServo.setPosition(servoPosition);
        leftServo = hardwareMap.get(Servo.class,"leftservo");
        leftServo.setPosition(1.0);

        // Corresponds lift motor names to motor variables.
        liftMotor1  = hardwareMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");

        // Initializes motor directions.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        // Waits for driver to press play.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Runs until driver presses stop.
        while (opModeIsActive()) {
            dualLift(0.0);
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

            if (gamepad1.x && toggle) {
                toggle = false;
                sleep(500);
                redLED.setState(false);
                greenLED.setState(true);

            }
            else if(gamepad1.x && !toggle) {
                toggle = true;
                sleep(500);
                redLED.setState(true);
                greenLED.setState(false);

            }

            if (toggle) {
                leftFrontDrive.setPower(leftFrontPower / 2);
                rightFrontDrive.setPower(rightFrontPower / 2);
                leftBackDrive.setPower(leftBackPower / 2);
                rightBackDrive.setPower(rightBackPower / 2);
            } else if (!toggle) {
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

            if (gamepad2.y)
            {
                MoveLift(HIGH_HEIGHT);
            }
            else if (gamepad2.x)
            {
                MoveLift(MED_HEIGHT);
            }
            else if (gamepad2.b)
            {
                MoveLift(LOW_HEIGHT);
            }
            else if (gamepad2.a){
                MoveLift(-20);
            }
            else if(gamepad2.dpad_up){
                MoveLift(Increment(10));
            }
            else if(gamepad2.dpad_down){
                MoveLift(Increment(-10));
            }

            else if(gamepad2.dpad_left){
                //Opens claw
                rightServo.setPosition(1.0);
                leftServo.setPosition(0.0);
                sleep(500);

                //drops lift
                MoveLift(0.0);
            }
            else if(gamepad2.right_bumper){
                //Opens claw
                rightServo.setPosition(1.0);
                leftServo.setPosition(0.0);

            }
            else if(gamepad2.left_bumper){
                //Closes claw
                rightServo.setPosition(0.0);
                leftServo.setPosition(1.0);

            }

            // Sends calculated power to motors.
            //leftFrontDrive.setPower(leftFrontPower);
            //rightFrontDrive.setPower(rightFrontPower);
            //leftBackDrive.setPower(leftBackPower);
            //rightBackDrive.setPower(rightBackPower);


            // Shows the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift Motors", "%5.2f", liftPower);
            telemetry.addData("precision mode",String.valueOf(toggle));
            telemetry.update();

            telemetry.addData("deviceName",sensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));

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
