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

@Autonomous(name ="BlueRedPlace", group = "A")
public class BlueRedPlace extends DriveMethods {
    public void runOpMode() {
        initMotorsSecondBot();

        waitForStart();

        driveForDistance(0.24, Direction.RIGHT, 0.5, 0);
        driveForDistance(0.3, Direction.FORWARD, 0.3, 0);
        driveForDistance(0.3, Direction.BACKWARD, 0.5, 0);
        driveForDistance(0.5, Direction.RIGHT, 0.5, 0);

        while(opModeIsActive()) {

        }
    }



}
