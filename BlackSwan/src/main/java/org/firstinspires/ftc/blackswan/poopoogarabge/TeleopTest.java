//package org.firstinspires.ftc.blackswan.poopoogarabge;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import static java.lang.Math.abs;
//
//import org.firstinspires.ftc.blackswan.Robot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@TeleOp(name="TeleopTest")
//
//public class TeleopTest extends LinearOpMode {
//    BNO055IMU imu;
//    double MAX_SPEED = 0.9;
//    Robot robot;
//    final int TICKS_PER_ROTATION = 537;
//
//    int GyroSensorVariable = 1;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        robot = new Robot(hardwareMap, telemetry, this);
//
//        DcMotor frontLeft, backLeft, frontRight, backRight;
//
//
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            double y = gamepad1.left_stick_y;
//            double x = gamepad1.right_stick_x;
//            double rx = gamepad1.left_stick_x;
//
//            backLeft.setPower(y + x + rx);
//            backRight.setPower(y - x + rx);
//            frontLeft.setPower(y + x - rx);
//            frontRight.setPower(y - x - rx);
//
//        };
//
//    }
//}