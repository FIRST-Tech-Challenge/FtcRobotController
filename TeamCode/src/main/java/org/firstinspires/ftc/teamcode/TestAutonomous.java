package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveMethods;
import static org.firstinspires.ftc.teamcode.Variables.*;
@Autonomous(name="TestStraightDriving", group="B")
public class TestAutonomous extends DriveMethods{
    public void runOpMode() {
        boolean calibrated = false;
        double previousZ = 0;
        double integratedZ = 0;
        BNO055IMU imu;
        DcMotor motorLinearSlide;

        initMotorsBlue();

        waitForStart();
        while (opModeIsActive()) {
            driveForDistance(3.3, Direction.FORWARD, 1.5, 0);
            driveForDistance(3.2, Direction.RIGHT, 1.5, 0);

            driveForDistance(3.2, Direction.BACKWARD, 1.5, 0);

            driveForDistance(3.2, Direction.LEFT, 1.5, 0);

        }
    }
}
