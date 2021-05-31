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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.WobbleSystem;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;

@TeleOp(name = "EchoOp")
public class EchoOp extends OpMode {
    private FtcDashboard dashboard;
    private GamePadController gamepad;
    private DriveTrain driveTrain;
    private FlyWheel flywheel;
    private Hitter hitter;
    private Motor intake1, intake2;
    private WobbleSystem wobbleSystem;
    private DifferentialDriveOdometry odometry;
    private RotationController rotationController;

    private double intakeSpeed = 0;
    private boolean wobbleHandOpen = true;
    private WobbleArmState wobbleArmState = WobbleArmState.UP;
    private boolean flywheelOn = false;
    private boolean flywheelPowershot = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        gamepad = new GamePadController(gamepad1);

        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                                    new Motor(hardwareMap, "dr"));
        driveTrain.resetEncoders();

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));

        odometry = new DifferentialDriveOdometry(new Rotation2d(rotationController.getAngleRadians()), new Pose2d(45,  -300, new Rotation2d(Math.PI/2)));

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);

        wobbleSystem = new WobbleSystem(new Motor(hardwareMap, "wobbleArmMotor"),
                                        hardwareMap.servo.get("wobbleArmServo"));

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);

        hitter = new Hitter(hardwareMap.servo.get("sv"));
    }

    @Override
    public void loop() {
        gamepad.update();
        double[] driveTrainDistance = driveTrain.getDistance();
        double leftDistanceInch = driveTrainDistance[0] / Vals.TICKS_PER_INCH_MOVEMENT;
        double rightDistanceInch = driveTrainDistance[1] / Vals.TICKS_PER_INCH_MOVEMENT;
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

        if(gamepad.isARelease()) {
            wobbleHandOpen = !wobbleHandOpen;
        }

        if(wobbleHandOpen) {
            wobbleSystem.hand_open();
        } else {
            wobbleSystem.hand_close();
        }

        if(gamepad.isUpRelease()) {
            wobbleArmState = WobbleArmState.UP;
        } else if(gamepad.isDownRelease()) {
            wobbleArmState = WobbleArmState.DOWN;
        } else if(gamepad.isLeftRelease()) {
            wobbleArmState = WobbleArmState.MID;
        }


        if(gamepad.isXRelease()) {
            flywheelPowershot = !flywheelPowershot;
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
            if(flywheelPowershot) {
                flywheel.on_slow();
            } else {
                flywheel.on();
            }
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

        switch (wobbleArmState) {
            case UP:
                wobbleSystem.arm_up();
                break;
            case MID:
                wobbleSystem.arm_mid();
                break;
            case DOWN:
                wobbleSystem.arm_down();
                break;
        }


        telemetry.addData("Flywheel Speed", flywheel.flywheel.get());
        telemetry.addData("Flywheel Velocity", flywheel.flywheel.getCorrectedVelocity());
        telemetry.addData("Flywheel Filtered Speed", Vals.flywheel_filtered_speed);
        telemetry.addData("Flywheel Position", flywheel.flywheel.getCurrentPosition());
        telemetry.addData("Hitter Position", hitter.hitter.getPosition());
        telemetry.addData("Left Speed", leftSpeed);
        telemetry.addData("Right Speed", rightSpeed);
        telemetry.addData("Left Distance", driveTrainDistance[0]);
        telemetry.addData("Right Distance", driveTrainDistance[1]);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Wobble Arm Position", wobbleSystem.wobbleArm.getCurrentPosition());
        telemetry.addData("Wobble Arm Rotations", wobbleSystem.wobbleArm.getCurrentPosition());
        telemetry.addData("Wobble Arm Distance", wobbleSystem.wobbleArm.getDistance());

        Pose2d pose = odometry.getPoseMeters();
        telemetry.addData("X Pos: ", pose.getX());
        telemetry.addData("Y Pos: ", pose.getY());
        telemetry.addData("Heading: ", pose.getHeading());


//        TelemetryPacket packet = new TelemetryPacket();
//        packet.fieldOverlay()
//                .setFill("blue")
//                .fillRect(pose.getX(), pose.getY(), 10, 10);
//        TelemetryPacket packet2 = new TelemetryPacket();
//        packet2.fieldOverlay()
//                .setFill("red")
//                .fillRect(0, 0, 30, 30);
//
//        dashboard.sendTelemetryPacket(packet);
//        dashboard.sendTelemetryPacket(packet2);


    }
}

enum WobbleArmState {
    DOWN,
    MID,
    UP
}