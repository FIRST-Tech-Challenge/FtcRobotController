package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.configuration.EditLynxModuleActivity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
@TeleOp(name="NewIMUTest", group="A")
@Disabled
public class IMUSampleCopy extends LinearOpMode {

    @Override
    public void runOpMode() {
        IntegratingGyroscope gyro;
        NavxMicroNavigationSensor navxMicro;
        ElapsedTime timer = new ElapsedTime();


        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;



        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.addLine("calibrated!");
        telemetry.update();

        // Wait until we're told to go
        waitForStart();

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        while (opModeIsActive()) {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addLine("Heading: " + angles.firstAngle);
            telemetry.update();


        }
    }

}
