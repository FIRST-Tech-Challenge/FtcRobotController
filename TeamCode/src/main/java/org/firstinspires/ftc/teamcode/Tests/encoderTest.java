package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Constants;
import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

@TeleOp
public class encoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Find a motor in the hardware map named "Arm Motor"
        Input input = new Input(hardwareMap);
        Motors motors = new Motors(hardwareMap);
        IMU imu = new IMU(hardwareMap);


        waitForStart();
        int position = 0;
        imu.SetYaw();

        while (opModeIsActive()) {

            input.move(gamepad1.left_stick_y * 100);
            input.spin(gamepad1.right_stick_x * 100);

            double yaw = imu.getAngle('y');

            double leftBackPosition = motors.getLeftFrontPosition();
            double rightBackPosition = motors.getRightFrontPosition();


            double CPR = 537.7;

            // Get the current position of the motor

            double lbrevolutions = leftBackPosition / CPR;
            double lbangle = lbrevolutions * 360;
            //double lbangleNormalized = lbangle % 360;

            double rbrevolutions = rightBackPosition / CPR;
            double rbangle = rbrevolutions * 360;
            //double rbangleNormalized = rbangle % 360;


            double rbdistance = Constants.WHEEL_CIRCUMFERENCE * rbrevolutions * Constants.FRICTION_PERCENT;
            double lbdistance = Constants.WHEEL_CIRCUMFERENCE * lbrevolutions * Constants.FRICTION_PERCENT;

            double distance;

            if(!input.isSpin()) {
                distance = Math.floor(((lbdistance + rbdistance))/2);
            }
            else {
                distance = (lbdistance - rbdistance)/2;
            }




            // Show the position of the motor on telemetry
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Encoder Revolutions", rbdistance);
            telemetry.addData("Encoder Revolutions", lbdistance);
//                telemetry.addData("Encoder Angle (Degrees)", r\);
//                telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);
            telemetry.addData("Encoder Distance (in inches)", distance / 25.4);
            telemetry.addData("imu yaw", yaw);
            telemetry.update();
        }
    }
}
