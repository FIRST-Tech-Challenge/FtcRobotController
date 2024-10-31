package org.firstinspires.ftc.teamcode.COMPETITIONCODE;

import com.parshwa.drive.auto.AutoDriverBetaV1;
import com.parshwa.drive.auto.Directions;
import com.parshwa.drive.tele.Drive;
import com.parshwa.drive.tele.DriveModes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

@Autonomous(name = "auto")
public class AUTO extends LinearOpMode {
    private AutoDriverBetaV1 autoDriver = new AutoDriverBetaV1();
    private Drive driver = new Drive();
    @Override
    public void runOpMode() throws InterruptedException {
        File ThreadManger = AppUtil.getInstance().getSettingsFile("ThreadManger.txt");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.UP);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        driver.change(imu);
        driver.change("RFM","RBM","LFM","LBM");
        driver.change(DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE);
        driver.init(hardwareMap,telemetry, DriveModes.MecanumFeildOriented);
        autoDriver.init(hardwareMap,driver);
        //TODO: GET REAL VALUES
        //TODO: SET UP MULTIAUTOMODE
        //TODO: SET UP ROTATIONS
        //IMPORTANT: DO THE TODOS
        int sample1pos = autoDriver.lineTo(-850.0, 420.0,1.0);
        int netZone = autoDriver.lineTo(0.0,450.0,1.0);
        int sample2pos = autoDriver.lineTo(-850.0,435.0, 1.0);
        int forwardTest = autoDriver.lineTo(-1000.0,0.0,1.0);
        int strafeTest = autoDriver.lineTo(-1000.0, -1000.0, 1.0);
        int diagonalTest = autoDriver.lineTo(0.0,0.0,1.0);
        int rotateTest = autoDriver.rotateRobot(90, Directions.RightRotateDirection);
        telemetry.addLine("initilized");
        telemetry.update();
        waitForStart();
        boolean completed = false;
        /*while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(forwardTest);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(strafeTest);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(diagonalTest);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(rotateTest);
        }*/
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(sample1pos);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(netZone);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(sample2pos);
        }
        completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(netZone);
        }
        //ReadWriteFile.writeFile(ThreadManger, "STOP");

    }
}
