package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Joshua OmniDirectional")
public class OmnidirectionalJoshua extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private IMU imu;

    //shortcut sticks
    private double leftStick_x = gamepad1.left_stick_x; //'x'
    private double leftStick_y = gamepad1.left_stick_y; // 'y'
    private double rightStickRotate_x = gamepad1.right_stick_x; // 'rx'





    /*
           plane launcher notes:
           spring power to launch plane, spring attached to the end and expanding where it decompresses till rest
           servo to move down for plane launcher
           servo is to make the servo decompress and compress
           the servo unlatches then plane go
           starts up not at rest then ends fully down not at rest

           weight: more weight,
           concerns: plane surface,


     */

    @Override
    public void init() {

        leftStick_x = gamepad1.left_stick_x;
        leftStick_y = gamepad1.left_stick_y;

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        //field centric: direction is relative to the field not robot
        // imu: inertial measurement unit detects force, angular rate to track orientation & velocity
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

    } //init

    @Override
    public void loop() {
        omniDirection();
    }
    //loop


    public void omniDirection() {

        if (gamepad1.options)
            imu.resetYaw(); //recalibrate

        AngleUnit angleUnit = AngleUnit.RADIANS;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

        //Rotate the movement direction counter to the bot's rotation make robot move in right direction even when rotate
        double rotX = leftStick_x * Math.cos(-botHeading) - leftStick_y * Math.sin(-botHeading);
        double rotY = leftStick_x * Math.sin(-botHeading) + leftStick_y * Math.cos(-botHeading);

        rotX *= 1.1; //counteract imperfect straifing 1.1 may change

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickRotate_x), 1);
        //scales -1 to 1

        //OmniDirectional movement
        frontLeft.setPower((rotY + rotX + rightStickRotate_x) / denom);
        backLeft.setPower((rotY - rotX + rightStickRotate_x) / denom);
        frontRight.setPower((rotY - rotX - rightStickRotate_x) / denom);
        backRight.setPower((rotY + rotX - rightStickRotate_x) / denom);
    }
}