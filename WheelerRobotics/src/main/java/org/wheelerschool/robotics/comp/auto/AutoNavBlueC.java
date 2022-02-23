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
public class AutoNavBlueC extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();

    public void runOpMode() {
        meccanum.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();

    }
    private void executeAutomaticSequence1(){
        // should get 22

        // auto for near carousel
        //FILL IN THE NON NEAR CAROUSEL HERE WITH FLIPPED VALS

        // here ur facing the warehouse
        meccanum.motorDriveLeftEncoded(meccanum.NORMAL_SPEED, 800);
        // <-
        meccanum.motorDriveBackEncoded(meccanum.NORMAL_SPEED, 800);
        meccanum.delay(1000);
        // \/
        meccanum.spinnySpinTime(meccanum.OPTIMAL_SPINNER_POWER, 1000);
        meccanum.delay(1000);
        // *
        meccanum.motorDriveRightEncoded(meccanum.NORMAL_SPEED, 775);
        meccanum.delay(1000);
        // ->



    }



}
