package org.firstinspires.ftc.teamcode.Autos;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Utility.Config.TURNING_P_GAIN;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "RedPark", group = "RedAutos")
public class RedPark extends LinearOpMode {
    /* Declare OpMode members. */

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private IMU imu;

    // TODO: change this number once we do encoder math for the correct number.
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH_FIELD_CALIBRATION = 12/29.0;
    public double COUNTS_PER_INCH = COUNTS_PER_INCH_FIELD_CALIBRATION*((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415));
    public void runOpMode() {
        FLDrive = hardwareMap.dcMotor.get("FLDrive");
        FRDrive = hardwareMap.dcMotor.get("FRDrive");
        BLDrive = hardwareMap.dcMotor.get("BLDrive");
        BRDrive = hardwareMap.dcMotor.get("BRDrive");

        imu = hardwareMap.get(IMU.class, "IMU");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();

        BLDrive.setMode(RUN_WITHOUT_ENCODER);
        BRDrive.setMode(RUN_WITHOUT_ENCODER);
        FLDrive.setMode(RUN_WITHOUT_ENCODER);
        BRDrive.setMode(RUN_WITHOUT_ENCODER);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.setAutoClear(false);

        waitForStart();

        double heading = getHeading() + 90.0;
        double speed = .2;

        while(((getHeading() > (heading- 5)) || (getHeading() < (heading + 5))))
        {
            FLDrive.setPower(-speed);
            FRDrive.setPower(speed);
            BLDrive.setPower(-speed);
            BRDrive.setPower(speed);
            telemetry.addData("Heading", getHeading());
            //addTelemetry(telemetry);
        }
        // Stop all motion;
        //autoDriveForward(0, 0);
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);

        //This path starts on the left of the teeth of the tile next to the last bit of red tape for the observation zone.

        //autoDriveForward(.3,3);
        //Negative is to the right because of the way the IMU faces.
        //turnToAngle(.2, 90.0);
        //Works
        //autoDriveForward(.3,24);
        //Doesn't work.
        //turnToAngle(.2, 90, telemetry);
        //Works
        //autoDriveForward(.3, -3);
    }

    /*
        *  Method to perform a relative move, based on encoder counts.
        *  Encoders are not reset as the move is based on the current position.
        *  Move will stop if any of three conditions occur:
        *  1) Move gets to the desired position
        *  2) Move runs out of time
        *  3) Driver stops the OpMode running.
    */
    public void autoDriveForward(double speed, double inchesForward) {
        int newFLTarget = 0;
        int newFRTarget = 0;
        int newBLTarget = 0;
        int newBRTarget = 0;

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);


        if (opModeIsActive()) {
            //Resetting encoders
            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFLTarget = FLDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            newFRTarget = FRDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            newBLTarget = BLDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            newBRTarget = BRDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            FLDrive.setTargetPosition(newFLTarget);
            FRDrive.setTargetPosition(newFRTarget);
            BLDrive.setTargetPosition(newBLTarget);
            BRDrive.setTargetPosition(newBRTarget);

            // Determine new target position, and pass to motor controller

            // Turn On RUN_TO_POSITION
            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLDrive.setPower(Math.abs(speed));
            FRDrive.setPower(Math.abs(speed * .85));
            BLDrive.setPower(Math.abs(speed));
            BRDrive.setPower(Math.abs(speed * .85));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy()) && telemetry != null) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newFLTarget, newFRTarget);
                telemetry.addData("Currently at", " at %7d :%7d", FLDrive.getCurrentPosition(), FRDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            //Turn off RUN_TO_POSITION
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    /**
     * For auto. Turns to a specified angle, without resetting the heading.
     *
     * @param heading    How many degrees to turn. [-180, 180].
     * @param speed     How fast to turn. (0, 1].
     * @see org.firstinspires.ftc.teamcode.Hardware.Drivebase#driveSideways
     * @see org.firstinspires.ftc.teamcode.Hardware.Drivebase#autoDriveForward
     * @see org.firstinspires.ftc.teamcode.Hardware.Drivebase#turnToAngle
     */
    public void turnToAngle(double speed, double heading) {


        //FLDrive.setDirection(DcMotor.Direction.FORWARD);
        //FRDrive.setDirection(DcMotor.Direction.REVERSE);
        //BLDrive.setDirection(DcMotor.Direction.FORWARD);
        //BRDrive.setDirection(DcMotor.Direction.REVERSE);

        //Give it a tolerence so it doesn't oscilate and give a negative power intead of reversing directions.
        /*while((opModeIsActive() && (getHeading() < heading - 1) && getHeading() > heading + 1)) {
            FLDrive.setPower(speed);
            FRDrive.setPower(-speed);
            BLDrive.setPower(speed);
            BRDrive.setPower(-speed);
        } */

    }

    /**
     * Ensures that an angle is within [-180, 180].
     *
     * @param n Angle in degrees.
     * @return Angle wrapped into [-180, 180].
     */
    private double wrapAngle(double n) {
        double x = (n % 360 + 360) % 360;
        return x < 180 ? x : x - 360;
    }

    private double getTurningCorrection(double angle) {
        return Range.clip(wrapAngle(angle - getHeading()) * TURNING_P_GAIN, -1, 1);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param FLDrivePower Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param FRDrivePower Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param BLDrivePower
     * @param BRDrivePower
     */
    public void setDrivePowers(double FLDrivePower, double FRDrivePower, double BLDrivePower, double BRDrivePower) {
        // Output the values to the motor drives.
        FLDrive.setPower(FLDrivePower);
        FRDrive.setPower(FRDrivePower);
        BLDrive.setPower(BLDrivePower);
        BRDrive.setPower(BRDrivePower);
    }

    public void setDrivePowers(double power) {
        // Output the values to the motor drives.
        FLDrive.setPower(power);
        FRDrive.setPower(power);
        BLDrive.setPower(power);
        BRDrive.setPower(power);
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        FLDrive.setMode(mode);
        FRDrive.setMode(mode);
        BLDrive.setMode(mode);
        BRDrive.setMode(mode);
    }

    /**
     * @return The robot heading in degrees.
     */
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void addTelemetry(Telemetry telemetry) {
        // Show the wheel powers.
       //telemetry.addData("FLDrive", "%4.2f, %4.2f", FLDrive.getCurrentPosition());
       //telemetry.addData("FRDrive", "%4.2f, %4.2f", FRDrive.getCurrentPosition());
       //telemetry.addData("BLDrive", "%4.2f, %4.2f", BLDrive.getCurrentPosition());
       //telemetry.addData("BRDrive", "%4.2f, %4.2f", BRDrive.getCurrentPosition());
       //Speed.
       //telemetry.addData("FLSpeed", "%4.2f, %4.2f", FLDrive.getPower());
       //telemetry.addData("FRSpeed", "%4.2f, %4.2f", FRDrive.getPower());
       //telemetry.addData("BLSpeed", "%4.2f, %4.2f", BLDrive.getPower());
        //telemetry.addData("BRSpeed", "%4.2f, %4.2f", BRDrive.getPower());

        telemetry.addData("Heading", "%4.2f, %4.2f", getHeading());
    }
}


