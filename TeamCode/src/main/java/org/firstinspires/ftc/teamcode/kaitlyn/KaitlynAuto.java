package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.kaitlyn.KaitlynRobot;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp
public class KaitlynAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        //encoderMove(12);
        //mecanumStrafe(12);
        KaitlynRobot robot = new KaitlynRobot(hardwareMap, telemetry, this);
        robot.encoderMoveWithPID(12);
        wait(2);
        robot.encoderMoveWithPID(-12);
//        wait(1);
//        robot.mecanumStrafe(8);

    }

    public void wait(int seconds) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (true) {

            if (elapsedTime.milliseconds() >= seconds*1000) {
                break;
            }

        }
    }
}

