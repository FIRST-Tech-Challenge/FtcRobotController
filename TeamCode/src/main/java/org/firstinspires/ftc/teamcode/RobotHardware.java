package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * See org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.RobotHardware for the FTC example
 */
public class RobotHardware {
    private OpMode opMode = null;

    private DcMotor frontLeftDrive   = null;
    private DcMotor frontRightDrive  = null;

    private DcMotor backLeftDrive   = null;
    private DcMotor backRightDrive  = null;

    private Servo spendexerWheel = null;

    private IMU imu = null;
    private IMU.Parameters parameters = null;

    public RobotHardware(OpMode opMode1) {opMode = opMode1;}

    private DcMotor getDcMotor(String id) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, id);
        return motor;
    }

    private Servo getServo(String id) {
        Servo servo = opMode.hardwareMap.get(Servo.class, id);
        return servo;
    }

    private IMU getImu(String id) {
        final IMU imu2 = opMode.hardwareMap.get(IMU.class, id);
        return imu2;
    }

    private void _init() throws Exception {
        // set up motors
        frontLeftDrive = this.getDcMotor("frontLeft");
        frontRightDrive = this.getDcMotor("frontRight");
        backLeftDrive = this.getDcMotor("backLeft");
        backRightDrive = this.getDcMotor("backRight");
        spendexerWheel = this.getServo("pollen");

        // set up imu
        imu = getImu("imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // set directions

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        spendexerWheel.setDirection(Servo.Direction.FORWARD);
    }

    public double getHeading(){
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void setDrivePower(double left, double right) {
        // set left drive
        frontLeftDrive.setPower(left);
        backLeftDrive.setPower(left);

        // set right drive
        frontRightDrive.setPower(right);
        backRightDrive.setPower(right);
    }

    public void setArmPower(double power) {
        // stub, fill in later
        this.setPollenWheelPosition(power);
    }

    public void setPollenWheelPosition(double angle) {
        spendexerWheel.setPosition((angle - 180) / 360);
    }

    public double getPollenWheelPosition(){
        double rawPosition = spendexerWheel.getPosition();
        double _360degPosition = rawPosition * 360;
        double _180degPosition;
        _180degPosition = rawPosition - 180;

        return _180degPosition;
    }

    public void driveRobot(double drive, double turn) {
        double left = drive + turn;
        double right = drive - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        setDrivePower(left, right);
    }

    /**
     *
     * @param offset
     */
    public void setHandPositions (double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        // set positions, here is the ftc sample code
        /*
         leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
         */
    }

    public void init() throws RobotHardwareInitializationFailed{
        opMode.telemetry.addData(">", "Initializing hardware...");
        try {
            this._init();
        } catch (Exception e) {
            opMode.telemetry.addData(">", "Hardware initialization FAILED!");
            throw new RobotHardwareInitializationFailed(e.getMessage() + "\n" + e.getStackTrace().toString());
        }
    }
}
