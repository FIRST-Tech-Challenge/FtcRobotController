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
        int hangSpeciminPos = autoDriver.lineTo(1000.0, 0.0,1.0);
        int moveToSample1 = autoDriver.lineTo(1000.0,-1000.0,1.0);
        int rotateToCollectSample1 = autoDriver.rotateRobot(180.0, Directions.RightRotateDirection);
        int moveToNetZone1 = autoDriver.lineTo(10.0,100.0, 1.0);
        int moveOutOfNetZone1 = autoDriver.lineTo(50.0,150.0,1.0);
        telemetry.addLine("initilized");
        telemetry.update();
        waitForStart();
        boolean completed = false;
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(hangSpeciminPos);
        }
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(moveToSample1);
        }
        while(!isStopRequested() && !completed){
            Pose2D pos = autoDriver.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            completed = autoDriver.move(rotateToCollectSample1);
        }
        ReadWriteFile.writeFile(ThreadManger, "STOP");
    }
}
