package org.firstinspires.ftc.teamcode.COMPETITIONCODE;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.parshwa.drive.auto.AutoDriverBetaV1;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.COMPETITIONCODE.data.SliderManger;
import org.firstinspires.ftc.teamcode.COMPETITIONCODE.data.servoManger;

import java.util.Locale;

@Autonomous(name = "left auto", preselectTeleOp = "teleop")
public class LEFTAUTO extends LinearOpMode {
    private AutoDriverBetaV1 autoDriver = new AutoDriverBetaV1();
    private Drive driver = new Drive();

    private SliderManger SM = new SliderManger();
    private DcMotor sc, sr;
    private servoManger clawServo = new servoManger();
    private servoManger clawRotateServo = new servoManger();
    private servoManger clawRotateServo2 = new servoManger();
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
        IMU imu = hardwareMap.get(IMU.class, "imu");
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
        int dropSample1 = autoDriver.lineTo(-425.0,1381.5,1.0);
        int pickupSample2mid = autoDriver.lineTo(-600.0,1000.0,1.0);
        int pickupSample2 = autoDriver.lineTo(-700.0,1000.0,1.0);
        int dropSample2 = autoDriver.lineTo(-425.0,1381.5,1.0);
        int pickupSample3 = autoDriver.lineTo(-700.0,1500.0,1.0);
        int dropSample3 = autoDriver.lineTo(-900.0,1500.0,1.0);
        int pickupSample4 = autoDriver.lineTo(-700.0,1500.0,1.0);
        int dropSample4 = autoDriver.lineTo(-900.0,1500.0,1.0);
        telemetry.addLine("initilized");
        telemetry.update();
        waitForStart();
        clawServo.setServoPosition(0.0);
        clawRotateServo.setServoPosition(0.7);
        clawRotateServo2.setServoPosition(0.55);
        boolean completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(dropSample1);
        }
        clawRotateServo.setServoPosition(0.4);
        safeWaitSeconds(0.5);
        int RotateTarget = 1015;
        while(!isStopRequested() && !(Math.abs(RotateTarget-sr.getCurrentPosition()) <= 10)){
            SM.setPos(RotateTarget);
        }
        int target = -3000;
        while(!isStopRequested() && !(Math.abs(target-sc.getCurrentPosition()) <= 10)){
            SM.setPos2(target);
        }
        clawRotateServo.setServoPosition(0.45);
        safeWaitSeconds(0.5);
        clawServo.setServoPosition(0.39);
        safeWaitSeconds(0.5);
        clawRotateServo.setServoPosition(0.05);
        safeWaitSeconds(0.5);
        target = 0;
        while(!isStopRequested() && !(Math.abs(target-sc.getCurrentPosition()) <= 10)){
            SM.setPos2(target);
        }
        RotateTarget = 0;
        while(!isStopRequested() && !(Math.abs(RotateTarget-sr.getCurrentPosition()) <= 10)){
            SM.setPos(RotateTarget);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(pickupSample2mid);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(pickupSample2);
        }
        clawServo.setServoPosition(0.0);
        safeWaitSeconds(0.5);
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(dropSample2);
        }
        clawRotateServo.setServoPosition(0.4);
        safeWaitSeconds(0.5);
        RotateTarget = 1015;
        while(!isStopRequested() && !(Math.abs(RotateTarget-sr.getCurrentPosition()) <= 10)){
            SM.setPos(RotateTarget);
        }
        target = -3000;
        while(!isStopRequested() && !(Math.abs(target-sc.getCurrentPosition()) <= 10)){
            SM.setPos2(target);
        }
        clawRotateServo.setServoPosition(0.45);
        safeWaitSeconds(0.5);
        clawServo.setServoPosition(0.39);
        safeWaitSeconds(0.5);
        clawRotateServo.setServoPosition(0.05);
        safeWaitSeconds(0.5);
        target = 0;
        while(!isStopRequested() && !(Math.abs(target-sc.getCurrentPosition()) <= 10)){
            SM.setPos2(target);
        }
        RotateTarget = 0;
        while(!isStopRequested() && !(Math.abs(RotateTarget-sr.getCurrentPosition()) <= 10)){
            SM.setPos(RotateTarget);
        }
        while(!isStopRequested() && !gamepad1.a){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine(data);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(pickupSample3);
        }
        while(!isStopRequested() && !gamepad1.a){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine(data);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(dropSample3);
        }
        while(!isStopRequested() && !gamepad1.a){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine(data);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(pickupSample4);
        }
        while(!isStopRequested() && !gamepad1.a){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine(data);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(dropSample4);
        }
        while(!isStopRequested() && !gamepad1.a){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine(data);
        }
    }
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}
