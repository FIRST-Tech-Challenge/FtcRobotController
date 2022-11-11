package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Variables.*;

import android.graphics.drawable.GradientDrawable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name ="RRLow", group = "A")
public class RedRedPlace extends DriveMethods {
    public void runOpMode() {
        initMotorsBlue();


        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.2, Direction.RIGHT, 0.3, 0);
        driveForDistance(0.28, Direction.FORWARD, 0.3, 0);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.28, Direction.BACKWARD, 0.5, 0);

        GoToHeight(0);

        driveForDistance(1.4, Direction.LEFT, 0.5, 0);


        while(opModeIsActive()) {

        }
    }



}
