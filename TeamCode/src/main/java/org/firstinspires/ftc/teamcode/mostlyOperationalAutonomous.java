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

@Autonomous(name ="MOAuto", group = "A")
public class mostlyOperationalAutonomous extends DriveMethods {
    @Override
    public void runOpMode() {
        BNO055IMU imu;

        initMotorsBlue();

        //calibrateImu();

        waitForStart();


        while(opModeIsActive()) {

        }
    }
}
