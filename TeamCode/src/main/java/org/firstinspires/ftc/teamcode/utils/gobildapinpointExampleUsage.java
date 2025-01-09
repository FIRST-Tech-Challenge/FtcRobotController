//package org.firstinspires.ftc.teamcode.utils;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.roadrunner.tuning.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.subsystems.Imu;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import java.util.Locale;
//
//public class gobildapinpointExampleUsage extends LinearOpMode {
//    private Imu imu;
//    GoBildaPinpointDriver odometry;
//    double oldTime = 0;
//    int headingOffsetDeg = 0;
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//        ));
//
//        imu.initialize(parameters);
//
//        //setting up odometry
//        odometry.setOffsets(-84.0, -168.0, headingOffsetDeg);
//        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odometry.recalibrateIMU();
//        odometry.resetPosAndIMU();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", odometry.getXOffset());
//        telemetry.addData("Y offset", odometry.getYOffset());
//        telemetry.addData("Device Version Number:", odometry.getDeviceVersion());
//        telemetry.addData("Device Scalar", odometry.getYawScalar());
//        while(opModeIsActive())
//        {
//            odometry.update();
//            /*
//            This code prints the loop frequency of the REV Control Hub. This frequency is effected
//            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
//            of time each cycle takes and finds the frequency (number of updates per second) from
//            that cycle time.
//             */
//            double newTime = getRuntime();
//            double loopTime = newTime - oldTime;
//            double frequency = 1 / loopTime;
//            oldTime = newTime;
//
//             /*
//            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
//             */
//            Pose2D pos = odometry.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//
//            /*
//            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
//             */
//            Pose2D vel = odometry.getVelocity();
//            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Velocity", velocity);
//
//             /*
//            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
//            READY: the device is working as normal
//            CALIBRATING: the device is calibrating and outputs are put on hold
//            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
//            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
//            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
//            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
//            */
//            telemetry.addData("Status", odometry.getDeviceStatus());
//
//            telemetry.addData("Pinpoint Frequency", odometry.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
//
//            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
//            //Driver controls
//        }
//    }
//}
