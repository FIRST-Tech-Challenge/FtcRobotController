package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Timer;

@Autonomous(name="chassisTest", group="Linear Opmode2")
public class chassisTest extends LinearOpMode {

    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;



    Orientation myRobotOrientation;

    @Override
    public void runOpMode() {

        bl = hardwareMap.get(DcMotorEx.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");

        //setup
        telemetry.setAutoClear(false);
        // initialize hardware



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            fl.setPower(0.2);
            fr.setPower(0.2);
            bl.setPower(0.2);
            br.setPower(0.2);
            sleep(2000);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            sleep(1000);

            fl.setPower(-0.2);
            fr.setPower(-0.2);
            bl.setPower(-0.2);
            br.setPower(-0.2);
            sleep(2000);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            sleep(1000);

            fl.setPower(-0.4);
            fr.setPower(0.4);
            bl.setPower(0.4);
            br.setPower(-0.4);
            sleep(2000);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            sleep(1000);

            fl.setPower(0.4);
            fr.setPower(-0.4);
            bl.setPower(-0.4);
            br.setPower(0.4);
            sleep(2000);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            sleep(1000);




            sleep(9000000);

        }
    }


}

