package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.copySign;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Button;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotorEx lift = null;
    private IMU imu = null;
    private Servo servo = null;
    // private Servo crossbow = null;
    double powercoef = 0.5;

    private Button upButton = new Button();
    private Button downButton = new Button();
    private int liftSetpoint = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        imu = hardwareMap.get(IMU.class, "imu");

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");

        servo = hardwareMap.get(Servo.class, "servo");
        // crossbow = hardwareMap.get(Servo.class, "crossbow");

        leftFrontDrive.setDirection(Constants.motorDirections.get("left_front"));
        leftBackDrive.setDirection(Constants.motorDirections.get("left_back"));
        rightFrontDrive.setDirection(Constants.motorDirections.get("right_front"));
        rightBackDrive.setDirection(Constants.motorDirections.get("right_back"));
        lift.setDirection(Constants.motorDirections.get("lift"));

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu.resetYaw();

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Lift joystick position: ", gamepad2.right_stick_y);
        }

        runtime.reset();
        servo.setPosition(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // arm servo debug code
            // arm.setPower(gamepad2.right_stick_y * 1/4);
            // servo.setPosition(position);
            // intake.setPower(-gamepad2.right_trigger);

            // crossbow
            /*
            double crossbowSetpoint;
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                telemetry.addLine("crossbowing!!");
                crossbowSetpoint = 0.1;
            } else {
                crossbowSetpoint = 0.2;
            }
            telemetry.addData("crossbow setpoint", crossbowSetpoint);
            if (crossbowSetpoint != crossbow.getPosition()) crossbow.setPosition(crossbowSetpoint);

             */

            if (gamepad1.b) imu.resetYaw();

            handleLift();

            HandleDrivetrain();
            telemetry.update();
        }
    }


    /*

    if trigger held set arm position to -33 and servo position to 0.84
    if joystick up and position between than -33 and 0, set servo to 0.95
    when position greater than 0 set servo to 0.15
    when scoring set position to 0.7
    */

    public void handleLift() {
        double servoSetpoint;
        int liftPos = lift.getCurrentPosition();
        telemetry.addData("lift position: ", liftPos);
        /*
        double liftPower = 0;
        if (liftPos > Constants.elevatorPositionTop && -gamepad2.right_stick_y > 0) liftPower = gamepad2.right_stick_y;
        else if (liftPos < Constants.elevatorPositionBottom && -gamepad2.right_stick_y < 0) liftPower = gamepad2.right_stick_y;
        else liftPower = gamepad2.right_stick_y;
        lift.setVelocity(liftPower * 1500 + 1);
         */
        lift.setVelocity(gamepad2.right_stick_y * 1500 + 1);
        intake.setPower(-gamepad2.right_trigger);
        telemetry.addData("gamepad2 y:" , gamepad2.y);
        telemetry.addData("gamepad2 dpad up: ", gamepad2.dpad_up);
        telemetry.addData("gamepad2 right joystick y: ", gamepad2.right_stick_y);
        // TODO: tilt down for intaking
        if (gamepad2.a) {
            servoSetpoint = 0.19;// 0.1 worked aright 0.13 works better
        } else if (gamepad2.right_trigger > 0.5){
            servoSetpoint = 0.00;
        } else {
            servoSetpoint = 0.1;
        }
        sleep(20);
        telemetry.addData("servoSetpoint: ", servoSetpoint);
        telemetry.addData("Last servo setpoint: ", servo.getPosition());
        if (servo.getPosition() != servoSetpoint) servo.setPosition(servoSetpoint);
    }

    public void handleLiftSetpoints() {
        upButton.update(gamepad1.dpad_up);
        downButton.update(gamepad1.dpad_down);
        if (upButton.is(Button.State.TAP) && lift.getCurrentPosition() < Constants.elevatorPositionTop) liftSetpoint += 100;
        else if (downButton.is(Button.State.TAP) && lift.getCurrentPosition() > Constants.elevatorPositionBottom) liftSetpoint -= 100;
        lift.setTargetPosition(liftSetpoint);
        if (lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.5);
    }

    public void HandleDrivetrain() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        if(gamepad1.right_bumper) powercoef = 1;
        else powercoef = 0.4;

        double forward = -gamepad1.left_stick_y; /* Invert stick Y axis */
        double strafe = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        /*
        double gyro_radians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("gyro angle: ", gyro_radians);
        double temp = forward * cos(gyro_radians) +
                strafe * sin(gyro_radians);
        strafe = -forward * sin(gyro_radians) +
                strafe * cos(gyro_radians);
        strafe *= -1;
        forward = temp;

         */

        /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
        /* rotated by the gyro angle, and can be sent to drive system */

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = powercoef * (forward + strafe + yaw);
        double rightFrontPower = powercoef * (forward - strafe - yaw);
        double leftBackPower = powercoef * (forward - strafe + yaw);
        double rightBackPower = powercoef * (forward + strafe - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("left front pos: ", leftFrontDrive.getCurrentPosition());
        telemetry.addData("left back pos: ", leftBackDrive.getCurrentPosition());
        telemetry.addData("right front pos: ", rightFrontDrive.getCurrentPosition());
        telemetry.addData("right back pos: ", rightBackDrive.getCurrentPosition());

    }
}
