package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="Mecanum Auto Back Left", group="Robot")
public class MecanumAutoBackLeft extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.0;
    public static final double TURN_SPEED = 0.5;
    private MecanumRobotController robotController;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();
        // robotController.turnTo(45, TURN_SPEED);
//        robotController.distanceDrive(48, 45, DRIVE_SPEED);
//        robotController.sleep(10);

        robotController.turnTo(15, TURN_SPEED);
        robotController.distanceDrive(30, 15, DRIVE_SPEED);
        robotController.turnTo(150, TURN_SPEED);
        robotController.distanceDrive(36, 150, DRIVE_SPEED);
    }

    public void initialize() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BACKLEFT");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        IMU gyro = hardwareMap.get(IMU.class, "imu2");
        gyro.resetYaw();
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, this);
    }
}
