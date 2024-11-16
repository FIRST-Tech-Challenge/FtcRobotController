package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.Locale;

@TeleOp(name = "Competition Main", group = "TeleOp")
public class Competition extends LinearOpMode {

    private Limelight3A limelight;
    GoBildaPinpointDriver imu; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;

    @Override
    public void runOpMode() {

        //Limelight Setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        //GoBilda Odometry Pod Setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", imu.getXOffset());
        telemetry.addData("Y offset", imu.getYOffset());
        telemetry.addData("Device Version Number:", imu.getDeviceVersion());
        telemetry.addData("Device Scalar", imu.getYawScalar());
        telemetry.update();

        // Initialize the motors
        // Declare motor variables
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions (reverse left side if needed)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            //Limelight Data
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }

                //Odometry
                odo.update(); //Update odometry
                double newTime = getRuntime();
                double loopTime = newTime - oldTime;
                double frequency = 1 / loopTime;
                oldTime = newTime;
                Pose2D pos = imu.getPosition();
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Position", data);
                Pose2D vel = imu.getVelocity();
                String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Velocity", velocity);
                telemetry.addData("Status", imu.getDeviceStatus());
                telemetry.addData("Pinpoint Frequency", imu.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
                telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            }
            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x;  // Strafe
            double rotation = gamepad1.right_stick_x; // Rotation

            // Calculate power for each motor
            double leftFrontPower = y + x + rotation;
            double rightFrontPower = y - x - rotation;
            double leftBackPower = y - x + rotation;
            double rightBackPower = y + x - rotation;

            // Normalize power values to keep them between -1 and 1
            double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
            maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            maxPower = Math.max(maxPower, Math.abs(rightBackPower));

            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;

            // Set power to the motors
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            // Telemetry for debugging
            telemetry.addData("Front Left Power", leftFrontPower);
            telemetry.addData("Front Right Power", rightFrontPower);
            telemetry.addData("Rear Left Power", leftBackPower);
            telemetry.addData("Rear Right Power", rightBackPower);
            telemetry.update();
        }
    }
}