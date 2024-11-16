package org.firstinspires.ftc.teamcode.COMPETITIONCODE;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.parshwa.drive.auto.AutoDriverBetaV1;
import com.parshwa.drive.auto.GoBildaPinpointDriver;
import com.parshwa.drive.tele.Drive;
import com.parshwa.drive.tele.DriveModes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages.SliderManger;
import org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages.servoManger;

@Autonomous(name = "left auto rotation", preselectTeleOp = "teleop")
public class LeftAutoRotation extends LinearOpMode {
    private AutoDriverBetaV1 autoDriver = new AutoDriverBetaV1();
    private Drive driver = new Drive();

    private SliderManger SM = new SliderManger();
    private DcMotor sc, sr;
    private servoManger clawServo = new servoManger();
    private servoManger clawRotateServo = new servoManger();
    private servoManger clawRotateServo2 = new servoManger();

    private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        clawServo.init(hardwareMap, "cs");
        clawRotateServo.init(hardwareMap, "crs");
        clawRotateServo2.init(hardwareMap, "crs2");
        sc = hardwareMap.dcMotor.get("sc");
        sr = hardwareMap.dcMotor.get("sr");
        sc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SM.init(sc,sr);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        driver.change(imu);
        driver.change("RFM","RBM","LFM","LBM");
        driver.change(DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE);
        driver.init(hardwareMap,telemetry, DriveModes.MecanumRobotOriented);
        autoDriver.init(hardwareMap,driver);
        //TODO: GET REAL VALUES
        //TODO: SET UP MULTIAUTOMODE
        //TODO: SET UP ROTATIONS
        //IMPORTANT: DO THE TODOS
        int dropSample1 = autoDriver.lineTo(-410.0,1120.0,1.0);
        int pickupSample2mid = autoDriver.lineTo(-600.0,1025.0,1.0);
        int pickupSample2 = autoDriver.lineTo(-725.0,1025.0,1.0);
        int dropSample2 = autoDriver.lineTo(-400.0,1110.0,1.0);
        int pickupSample3 = autoDriver.lineTo(-700.0,1500.0,1.0);
        int dropSample3 = autoDriver.lineTo(-900.0,1500.0,1.0);
        int pickupSample4 = autoDriver.lineTo(-700.0,1500.0,1.0);
        int dropSample4 = autoDriver.lineTo(-900.0,1500.0,1.0);
        telemetry.addLine("initilized");
        telemetry.update();
        waitForStart();
        clawServo.setServoPosition(0.0);
        clawRotateServo.setServoPosition(0.4);
        clawRotateServo2.setServoPosition(0.55);

        driveToPos(-150, -200);
        turnAngle(-45);

    }
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
    public void turnAngle(double turnAngle) {
        double error, currentHeadingAngle, driveMotorsPower;
        imu.resetYaw();

        error = turnAngle;
        GoBildaPinpointDriver odo = autoDriver.getOdo();
        DcMotor frontLeftMotor = driver.getFrontLeftMotor();
        DcMotor backLeftMotor = driver.getBackLeftMotor();
        DcMotor frontRightMotor = driver.getFrontRightMotor();
        DcMotor backRightMotor = driver.getBackRightMotor();
        while (opModeIsActive() && ((error > 1) || (error < -1))) {
            odo.update();
            telemetry.addData("X: ", odo.getPosX());
            telemetry.addData("Y: ", odo.getPosY());
            telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            driveMotorsPower = error / 200;

            if ((driveMotorsPower < 0.2) && (driveMotorsPower > 0)) {
                driveMotorsPower = 0.2;
            } else if ((driveMotorsPower > -0.2) && (driveMotorsPower < 0)) {
                driveMotorsPower = -0.2;
            }

            // Positive power causes left turn
            frontLeftMotor.setPower(-driveMotorsPower);
            backLeftMotor.setPower(-driveMotorsPower);
            frontRightMotor.setPower(driveMotorsPower);
            backRightMotor.setPower(driveMotorsPower);

            currentHeadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turnAngle - currentHeadingAngle;
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    public void driveToPos(double targetX, double targetY) {
        GoBildaPinpointDriver odo = autoDriver.getOdo();
        DcMotor frontLeftMotor = driver.getFrontLeftMotor();
        DcMotor backLeftMotor = driver.getBackLeftMotor();
        DcMotor frontRightMotor= driver.getFrontRightMotor();
        DcMotor backRightMotor= driver.getBackRightMotor();
        odo.update();
        boolean telemAdded = false;

        while (opModeIsActive() && ((Math.abs(targetX - odo.getPosX()) > 40)
                || (Math.abs(targetY - odo.getPosY())) > 40)) {
            odo.update();

            double x = 0.001*(targetX - odo.getPosX());
            double y = -0.001*(targetY - odo.getPosY());

            double botHeading = odo.getHeading();

            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

            if (!telemAdded) {
                telemetry.addData("x: ", x);
                telemetry.addData("y: ", y);
                telemetry.addData("rotX: ", rotX);
                telemetry.addData("rotY: ", rotY);
            }

            if (Math.abs(rotX) < 0.15) {
                rotX = Math.signum(rotX) * 0.15;
            }

            if (Math.abs(rotY) < 0.15) {
                rotY = Math.signum(rotY) * 0.15;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (rotX + rotY) / denominator;
            double backLeftPower = (rotX - rotY) / denominator;
            double frontRightPower = (rotX - rotY) / denominator;
            double backRightPower = (rotX + rotY) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("X: ", odo.getPosX());
            telemetry.addData("Y: ", odo.getPosY());
            telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
