package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp", group = "Teleops")
public class teleop extends OpMode {

    @Override
    public void init(){

    }
    @Override
    public void loop() {
        File file;
        Scanner scan;
        String fileName = "/sdcard/FIRST/PathTest.txt";
        String position;
        int positions = 0;
        String[][] path;
        double dYdX;
        double totalDistance;
        double distance;
        double theta;
        double xCoordinate;
        double yCoordinate;
        // robot centric

        // declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backR");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        if (gamepad1.y) {

            file = new File(fileName);
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            while(scan.hasNextLine()){
                positions++;
                scan.nextLine() ;
            }
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            path = new String[positions][7];
            for (int i = 0; i < path.length; i++) {
                if (scan.hasNextLine()){
                    position = scan.nextLine();
                    position = position.replaceAll("[^0-9. -]", "");
                    path[i] = position.split(" ");
                }
                try{
                    dYdX = Double.parseDouble(path[i][1]);
                    totalDistance = Double.parseDouble(path[i][2]);
                    distance = Double.parseDouble(path[i][3]);
                    theta = Double.parseDouble(path[i][4]);
                    xCoordinate = Double.parseDouble(path[i][5]);
                    yCoordinate = Double.parseDouble(path[i][6]);
                } catch (NumberFormatException e){
                    continue;
                }
                telemetry.addLine("dYdX = " + dYdX);
                telemetry.addLine("totalDist = " + totalDistance);
                telemetry.addLine("dist = " + distance);
                telemetry.addLine("theta = " + theta);
                telemetry.addLine("x = " + xCoordinate);
                telemetry.addLine("y = " + yCoordinate);
                telemetry.addLine(" ");
            }
            telemetry.update();
        }


        // field centric activation
        if (gamepad1.b) {
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // change it to match the actual orientation of the rev control hub
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);



            // this would be start on Xbox (changeable)
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // rotate the movement direction to counter the robot's true orientation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


    }
        else {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

}

    @Override
    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }

}