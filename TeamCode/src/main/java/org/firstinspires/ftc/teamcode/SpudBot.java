package org.firstinspires.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**

The SpudBot singleton is the core of all of our robot code.
It contains the hardware objects and methods that are used by all of our opmodes.

**/
public class SpudBot {

    private static SpudBot robot;
    public static SpudBot get() {
        if(robot == null) {
            robot = new SpudBot();
        }
        return robot;
    }

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private SpudBot() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl_drv");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl_drv");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr_drv");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br_drv");
    }

    public void resetHardware() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}