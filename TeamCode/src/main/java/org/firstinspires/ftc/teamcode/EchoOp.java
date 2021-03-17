package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;

@TeleOp(name = "EchoOp")
public class EchoOp extends OpMode {
    FtcDashboard dashboard;
    private GamePadController gamepad;
    private DriveTrain driveTrain;
    private FlyWheel flywheel;
    private Hitter hitter;
    private Motor intake1, intake2;
    private Motor wobbleArm;
    private Servo wobbleHand;
    private DifferentialDriveOdometry odometry;
    RotationController rotationController;

    private double intakeSpeed = 0;
    private double wobbleHandPos = Vals.wobble_hand_close;
    private double wobbleArmVelocity = 0;
    private boolean flywheelOn = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        gamepad = new GamePadController(gamepad1);

        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                                    new Motor(hardwareMap, "dr"));
        driveTrain.resetEncoders();

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));

        odometry = new DifferentialDriveOdometry(new Rotation2d(rotationController.getAngleRadians()), new Pose2d(45,  0, new Rotation2d(0)));

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);

        wobbleHand = hardwareMap.servo.get("wobbleArmServo");
        wobbleArm = new Motor(hardwareMap, "wobbleArmMotor");

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);

        hitter = new Hitter(hardwareMap.servo.get("sv"));
    }

    @Override
    public void loop() {
        gamepad.update();
        double leftDistanceInch = driveTrain.driveLeft.getDistance() / Vals.TICKS_PER_INCH_MOVEMENT;
        double rightDistanceInch = driveTrain.driveRight.getDistance() / Vals.TICKS_PER_INCH_MOVEMENT;
        odometry.update(new Rotation2d(rotationController.getAngleRadians()), leftDistanceInch, rightDistanceInch);


        double leftSpeed = gamepad1.left_stick_y;
        double rightSpeed = gamepad1.right_stick_y;
        if(gamepad1.right_trigger >= 0.1) {
            intakeSpeed = 0.7;
        } else if(gamepad1.left_trigger >= 0.1) {
            intakeSpeed = -0.7;
        } else {
            intakeSpeed = 0;
        }

        if(gamepad1.a) {
            wobbleHandPos = Vals.wobble_hand_open;
        } else {
            wobbleHandPos = Vals.wobble_hand_close;
        }

        if(gamepad1.dpad_up) {
            wobbleArmVelocity = Vals.wobble_arm_speed;
        } else if(gamepad1.dpad_down) {
            wobbleArmVelocity = -Vals.wobble_arm_speed;
        } else {
            wobbleArmVelocity = 0;
        }


        if(gamepad.isYRelease()) {
            flywheel.flipDirection();
        }

        if(gamepad.isBRelease()) {
            if(flywheel.isOn()) {
                flywheelOn = false;
            } else {
                flywheelOn = true;
            }
        }

        if(flywheelOn) {
            flywheel.on();
        } else {
            flywheel.off();
        }

        String isReady = "FLYWHEEL NOT READY";

        if(flywheel.isReady()) {
            isReady = "FLYWHEEL READY!";
        }


        if(gamepad1.left_bumper) {
            hitter.hit();
        }
        else {
            hitter.reset();
        }

        driveTrain.setSpeed(leftSpeed, rightSpeed);

        intake1.set(intakeSpeed);
        intake2.set(intakeSpeed);

        wobbleHand.setPosition(wobbleHandPos);
        wobbleArm.set(wobbleArmVelocity);


        telemetry.addData("Flywheel Speed", flywheel.flywheel.get());
        telemetry.addData("Flywheel Velocity", flywheel.flywheel.getCorrectedVelocity());
        telemetry.addData("Flywheel Filtered Speed", Vals.flywheel_filtered_speed);
        telemetry.addData("Flywheel Position", flywheel.flywheel.getCurrentPosition());
        telemetry.addData("Hitter Position", hitter.hitter.getPosition());
        telemetry.addData("Left Speed", leftSpeed);
        telemetry.addData("Right Speed", rightSpeed);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Flywheel Ready State", isReady);

        Pose2d pose = odometry.getPoseMeters();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("blue")
                .fillRect(pose.getX(), pose.getY(), 40, 40);

        dashboard.sendTelemetryPacket(packet);


    }
}