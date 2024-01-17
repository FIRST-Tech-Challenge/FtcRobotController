package org.firstinspires.ftc.teamcode.TeleOps.Drivebases.Swerve.DiffySwerve.Aarush;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

@TeleOp (name="Diffy Go Burr")
public class DiffyTeleOp extends CommandOpMode {
    public DcMotorEx
            LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor;

    List<LynxModule> allHubs;
    AnalogInput leftAbsEncoder, rightAbsEncoder;

    public double multiplier = 0.98;

    public double rightAngle, leftAngle;
    double initialLeftAbsEncoderValue;
    double initialRightAbsEncoderValue;

    Translation2d rightPod, leftPod;
    SwerveDriveKinematics diffy;
    ChassisSpeeds speed;
    SwerveModuleState[] moduleStates;
    SwerveModuleState right, left;
    BNO055IMU.Parameters parameters;
    SwerveDrive rightModule, leftModule;

    @Override
    public void initialize() {
        allHubs = hardwareMap.getAll(LynxModule.class);

//        Bulk Caching "speeds" times up by fetching everything and storing that in cache
//        instead of blocking for each call and completely based on loop times for what I understand
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        leftAbsEncoder = hardwareMap.get(AnalogInput.class, "leftAbsoluteEncoder");
        rightAbsEncoder = hardwareMap.get(AnalogInput.class, "rightAbsoluteEncoder");

//        Initialize the stuff here otherwise things can get illegal, fast
        LeftFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fl"); //slot 1
        LeftBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "bl"); //slot 3
        RightFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fr"); //slot 2
        RightBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "br"); //slot 0

//        Instead of changing every single motor, we set all their properties at once through a list.
        List<DcMotorEx> motors = Arrays.asList(LeftFrontSwerveMotor, LeftBackSwerveMotor, RightBackSwerveMotor, RightFrontSwerveMotor);
//        Set the correct directions for this. Tune if needed IG
        LeftFrontSwerveMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Was forward
        LeftBackSwerveMotor.setDirection(DcMotorSimple.Direction.FORWARD); // Was reversed
        RightFrontSwerveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBackSwerveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        for(int i = 0; i < motors.size(); i++) {
//            Anything in here will get repeated for each individual "motor" in the list
            DcMotorEx motor = motors.get(i);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // TODO: Does this really improve or detract from anything?
        }

//        Initialize the other stuff here now
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);


        // Initializing everything in here just in case
        rightPod = new Translation2d(0, -0.115629);
        leftPod = new Translation2d(0, 0.115629);
        diffy = new SwerveDriveKinematics(rightPod, leftPod);

//        imu = imu1;
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);

        rightModule = new SwerveDrive(this.telemetry, RightFrontSwerveMotor, RightBackSwerveMotor);
        leftModule = new SwerveDrive(this.telemetry, LeftFrontSwerveMotor, LeftBackSwerveMotor);


        initialLeftAbsEncoderValue = leftAbsEncoder.getVoltage();
        initialRightAbsEncoderValue = rightAbsEncoder.getVoltage();

        telemetry.addLine("Initialized");
        telemetry.update();
    }
    @Override
    public void run() {
        super.run();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        // Implementing deadzone
        double joystickDeadzone = 0.1; // Adjust the threshold as needed
        if (Math.abs(x) < joystickDeadzone) {
            x = 0;
        }
        if (Math.abs(y) < joystickDeadzone) {
            y = 0;
        }
        if (Math.abs(rx) < joystickDeadzone) {
            rx = 0;
        }
        if (Math.abs(ry) < joystickDeadzone) {
            ry = 0;
        }

        // Check if the "A" button is pressed
        if (gamepad1.a) {
            // Rotate in place
            drive(0, 0, 1, 0); // You might need to adjust the values here based on your configuration
        } else {
            // Drive normally
            drive(x, y, rx, ry);
        }

        double leftAbsEncoderValue = leftAbsEncoder.getVoltage() - initialLeftAbsEncoderValue;
        double rightAbsEncoderValue = rightAbsEncoder.getVoltage() - initialRightAbsEncoderValue;

        // Print adjusted absolute encoder values to telemetry
        telemetry.addData("Left Absolute Encoder", leftAbsEncoderValue);
        telemetry.addData("Right Absolute Encoder", rightAbsEncoderValue);


        telemetry.update();

    }


    public void drive(double x, double y, double rx, double ry) {
//        speed = new ChassisSpeeds(1.680972, 1.680972, 1.680972);
        speed = new ChassisSpeeds(y, x, rx);
        moduleStates = diffy.toSwerveModuleStates(speed);
        right = moduleStates[0];
        left = moduleStates[1];

        double headingOffset = 0;

//        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        double heading = AngleUnit.normalizeRadians(orientation.firstAngle - headingOffset);

        Vector2d vec = new Vector2d(x, y);//.rotated(-heading); // Gives the angle that it needs to go.

        double vecX = vec.getX(); // How much to move left or right
        double vecY = vec.getY(); // How much to move forward, or backward

//
//        double targetAngle = vec.angle();

        moduleStates = diffy.toSwerveModuleStates(new ChassisSpeeds(y, x, -rx));
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.680972);

//        r.moduleController(right.speedMetersPerSecond, 90-right.angle.getDegrees(), damper, rightAngle);
//        l.moduleController(left.speedMetersPerSecond, 90+left.angle.getDegrees(), damper, leftAngle);

//        double correctedTargetAngle = (3*Math.PI - vec.angle()) % (2*Math.PI); // Radians
//        double correctedTargetAngle = vec.angle(); // Radians

//        double theAngleMagnitude = vec.norm(); // Magnitude

        rightModule.moduleController(right.speedMetersPerSecond, right.angle.getRadians(), multiplier, rightAngle);
        leftModule.moduleController(left.speedMetersPerSecond, left.angle.getRadians(), multiplier, leftAngle);

//        telemetry.addData("Right Angle", Math.toDegrees(rightAngle));
//        telemetry.addData("Left Angle", Math.toDegrees(leftAngle));
        telemetry.addData("Actual Angle", vec.norm());
    }
}