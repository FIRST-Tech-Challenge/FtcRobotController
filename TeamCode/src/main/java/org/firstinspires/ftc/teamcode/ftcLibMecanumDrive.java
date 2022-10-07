/** package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ftcLibMecanumDrive extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static final boolean FIELD_CENTRIC = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        MecanumDrive drive = new MecanumDrive(
                new DcMotor(hardwareMap, "FrontLeft", DcMotor.GoBILDA.RPM_435),
                new DcMotor(hardwareMap, "FrontRight",DcMotor.GoBILDA.RPM_435),
                new DcMotor(hardwareMap, "BackLeft", DcMotor.GoBILDA.RPM_435),
                new DcMotor(hardwareMap, "BackRight", DcMotor.GoBILDA.RPM_435)
        );
        System.out.println("RunOnMode");
        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        RevIMU imu = new RevIMU(hardwareMap);
        imu.Init();

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {

            // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
            // These are related to the left stick x value, left stick y value, and
            // right stick x value respectively. These values are passed in to represent the
            // strafing speed, the forward speed, and the turning speed of the robot frame
            // respectively from [-1, 1].

            if (!FIELD_CENTRIC) {

                // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
                // will move the robot in the direction of its current heading. Every movement
                // is relative to the frame of the robot itself.
                //
                //                 (0,1,0)
                //                   /
                //                  /
                //           ______/_____
                //          /           /
                //         /           /
                //        /___________/
                //           ____________
                //          /  (0,0,1)  /
                //         /     ↻     /
                //        /___________/

                // optional fourth parameter for squared inputs
                System.out.println("Robot");
                drive.driveRobotCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        false
                );
            } else {

                // Below is a model for how field centric will drive when given the inputs
                // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
                // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
                // regardless of the heading.
                //
                //                   heading
                //                     /
                //            (0,1,0) /
                //               |   /
                //               |  /
                //            ___|_/_____
                //          /           /
                //         /           / ---------- (1,0,0)
                //        /__________ /

                // optional fifth parameter for squared inputs
                System.out.println("Field");
                drive.driveFieldCentric(
                       -driverOp.getLeftX(),
                        -driverOp.getLeftY(),
                        driverOp.getRightX(),
                        -imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }

        }
    }

}
*/