package org.firstinspires.ftc.teamcode.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.PID;

public class Impasta {
    // Hardware variables
    private final AHRS imu;
    private IMU.Parameters parameters;
    private DcMotor fl, fr, bl, br, leftSlide, rightSlide, Intake;
    private PID pid = new PID(0.008, 0, 0);


    // Constructor for Impasta class
    public Impasta(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor leftSlide, DcMotor rightSlide, DcMotor Intake, AHRS imu) {
        // Assigning hardware references to local variables
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.Intake = Intake;
        this.imu = imu;

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

//         Initializing IMU parameters
//        parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
//                )
//        );
//
//        imu.initialize(parameters);
    }

    public Impasta(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, AHRS imu) {
        // Assigning hardware references to local variables
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.imu = imu;

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

//        parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
//                )
//        );
//
//        imu.initialize(parameters);
    }

    // Method to reset IMU yaw angle
    public void reset() {
//        imu.resetYaw();
//        imu.close()
        imu.zeroYaw();
    }

    // Method to drive the robot based on gamepad input
    public void driveBaseRobot(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double flpwr = (y + x + rx) / denominator;
        double blpwr = (y - x + rx) / denominator;
        double frpwr = (y - x - rx) / denominator;
        double brpwr = (y + x - rx) / denominator;

        fl.setPower(flpwr);
        bl.setPower(blpwr);
        fr.setPower(frpwr);
        br.setPower(brpwr);
    }

    public void driveBaseField(double y, double x, double rx) {
        double botHeading = Math.toRadians(imu.getYaw());

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double flpwr = (rotY + rotX + rx) / denominator;
        double blpwr = (rotY - rotX + rx) / denominator;
        double frpwr = (rotY - rotX - rx) / denominator;
        double brpwr = (rotY + rotX - rx) / denominator;

        fl.setPower(flpwr);
        bl.setPower(blpwr);
        fr.setPower(frpwr);
        br.setPower(brpwr);
    }

    // Method to control intake motor
    public void intake(double ip) {
        Intake.setPower(ip);
    }

    //Slide Control via power
    public void slides(double power) {
        leftSlide.setPower(-power);
        rightSlide.setPower(power);
    }
}
