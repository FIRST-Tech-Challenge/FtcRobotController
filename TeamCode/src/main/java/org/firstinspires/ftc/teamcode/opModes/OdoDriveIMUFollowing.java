package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.imu.IMU;
import org.firstinspires.ftc.teamcode.movement.Mecanum;
import org.firstinspires.ftc.teamcode.odometry.DriveWheelOdometryWheel;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryThatusesIMUforHeading;
import org.firstinspires.ftc.teamcode.odometry.OdometryWheel;
import org.firstinspires.ftc.teamcode.pathFollow.Follower;
import org.firstinspires.ftc.teamcode.utility.pose;

import java.util.ArrayList;
import java.util.List;

// localization with drive wheel encoders and IMU heading
// no following; use controller to move
@TeleOp(name="2 Odometry + Working DriveWheel + IMU Path following", group="Autonomous")
public class OdoDriveIMUFollowing extends LinearOpMode {
    Mecanum mecanum;
    IMU imu;
    Odometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanum = new Mecanum(hardwareMap);
        imu = new IMU(hardwareMap, telemetry);
//        String pathFile = "src/main/java/org/firstinspires/ftc/teamcode/pathFollow/PathTXTs/testPath.txt";
        odometry = defaultConfiguration();
        Follower pathFollower = new Follower(mecanum, odometry, telemetry);


        waitForStart();

        odometry.start();
        pathFollower.start();


        while (opModeIsActive()){
            telemetry.addData("IMU Position", imu.getPosition());
//            telemetry.addData("Odo intial pose")
            telemetry.addData("Odometry Position", odometry.getPosition());
            telemetry.update();

            //mecanum.drive(gamepad1.right_stick_x/3, gamepad1.right_stick_y/3, -gamepad1.left_stick_x/3);
        }

        pathFollower.stop();
        odometry.end();
        pathFollower.stop();
    }

    Odometry defaultConfiguration(){
        //physical wheels
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor vertical = hardwareMap.get(DcMotor.class, "vertical");
        DcMotor horizontal = hardwareMap.get(DcMotor.class, "horizontal");


        //odometry wheels
        OdometryWheel frontRightOdo = new DriveWheelOdometryWheel(new pose(178.5,168,Math.PI/2), frontRight);
        OdometryWheel frontLeftOdo = new DriveWheelOdometryWheel(new pose(-178.5,168,Math.PI/2), frontLeft);
        OdometryWheel backRightOdo = new DriveWheelOdometryWheel(new pose(178.5,-168,Math.PI/2), backRight);
        OdometryWheel backLeftOdo = new DriveWheelOdometryWheel(new pose(-178.5,-168,Math.PI/2), backLeft);
        OdometryWheel verticalOdo = new DriveWheelOdometryWheel(new pose(-180,91,Math.PI/2), vertical);
        OdometryWheel horizontalOdo = new DriveWheelOdometryWheel(new pose(170,-190,0), horizontal);

        List<OdometryWheel> odometryWheels = new ArrayList<>();
        odometryWheels.add(frontLeftOdo);
        odometryWheels.add(frontRightOdo);
        odometryWheels.add(backLeftOdo);
        odometryWheels.add(backRightOdo);
        odometryWheels.add(verticalOdo);
        odometryWheels.add(horizontalOdo);

        // odometry system
        pose initial = new pose(0,0,Math.PI/2);
        return new OdometryThatusesIMUforHeading(imu, initial, odometryWheels);
    }
}
