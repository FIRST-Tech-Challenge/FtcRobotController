package org.wheelerschool.robotics.comp.auto;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.comp.chassis.Meccanum;

@Autonomous
public class AutoNavBlue extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();

    public void runOpMode() {
        meccanum.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();

    }
    private void executeAutomaticSequence1(){
        // should get 26

        // auto for near carousel
        // gotta replace 0 with tested vals
            meccanum.closeServoFull();
            // ()
            meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 775);
            // /\
            meccanum.motorSpinRightEncoded(meccanum.NORMAL_SPEED, 110);
            // ~>
            meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 10);
            // |\
            meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 300);
            // /\
            meccanum.openServoHalf();
            // (_
            meccanum.motorDriveBackEncoded(meccanum.NORMAL_SPEED, 300);
            // \/
            meccanum.motorSpinLeftEncoded(meccanum.NORMAL_SPEED, 110 + 90); // first spin + 90
            // <~
            meccanum.motorDriveLeftEncoded(meccanum.NORMAL_SPEED, 300);
            // <-
            meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 1000);
            // /\

    }



}
