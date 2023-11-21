//// Simple autonomous program that drives bot forward until end of period
//// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
//// right and keeps going. Demonstrates obstacle avoidance and use of the
//// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
//// simulate touch sensor press and supports left as well as right turn.
////
//// Also uses IMU to drive in a straight line when not avoiding an obstacle.
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BHI260IMU;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
////@TeleOp
////@Disabled
//public class StraightIMUTest extends LinearOpMode {
//    DcMotor fLeft;
//    DcMotor fRight;
//    DcMotor bLeft;
//    DcMotor bRight;
//    //TouchSensor touch;
//    //BNO055IMU imu;
//    IMU imu;
//    Orientation lastAngles = new Orientation();
//    double globalAngle;
//    double power = .30;
//    double correction;
//    boolean aButton, bButton, touched;
//
//    // called when init button is  pressed.
//    @Override
//    public void runOpMode() throws InterruptedException {
//        fLeft = hardwareMap.dcMotor.get("fLeft");
//        fRight = hardwareMap.dcMotor.get("fRight");
//        bLeft = hardwareMap.dcMotor.get("bLeft");
//        bRight = hardwareMap.dcMotor.get("bRight");
//
//        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//        ));
//
//        imu.initialize(parameters);
//
//        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
//
//        double yaw   = robotOrientation.getYaw(AngleUnit.DEGREES); //Yaw = rotation around Z-axis (points straight up through logo)
//        //double pitch = robotOrientation.getPitch(AngleUnit.DEGREES); // Pitch = rotation around X-axis (points toward right side I2C ports)
//        //double roll  = robotOrientation.getRoll(AngleUnit.DEGREES); // Roll = rotation around Y-axis (points towards top edge USB ports)
//
//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//
//
//        // wait for start button.
//        waitForStart();
//
//        telemetry.addData("Mode", "running");
//        telemetry.update();
//
//        sleep(1000);
//
//        // drive until end of period.
//
//        while (opModeIsActive())
//        {
//            // Use gyro to drive in a straight line.
//            correction = checkDirection();
//
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("3 correction", correction);
//            telemetry.update();
//
//            fLeft.setPower(power - correction);
//            fRight.setPower(power + correction);
//
//            // We record the sensor values because we will test them in more than
//            // one place with time passing between those places. See the lesson on
//            // Timing Considerations to know why.
//
////            aButton = gamepad1.a;
////            bButton = gamepad1.b;
////            touched = touch.isPressed();
//
////            if (touched || aButton || bButton)
////            {
////                // backup.
////                fLeft.setPower(power);
////                fRight.setPower(power);
////
////                sleep(500);
////
////                // stop.
////                fLeft.setPower(0);
////                fRight.setPower(0);
////                bLeft.setPower(0);
////                bRight.setPower(0);
//
//                // turn 90 degrees right.
//                //if (touched || aButton) rotate(-90, power);
//
//                // turn 90 degrees left.
//                //if (bButton) rotate(90, power);
////            }
//        }
//
//        // turn the motors off.
//        fLeft.setPower(0);
//        fRight.setPower(0);
//        bLeft.setPower(0);
//        bRight.setPower(0);
//    }
//
//    /**
//     * Resets the cumulative angle tracking to zero.
//     */
//    private void resetAngle()
//    {
//        imu.getRobotYawPitchRollAngles().getAcquisitionTime();
//        //lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    /**
//     * Get current cumulative angle rotation from last reset.
//     * @return Angle in degrees. + = left, - = right.
//     */
//    private double getAngle()
//    {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles;
//
//        //double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
////        if (deltaAngle < -180) {
////            deltaAngle += 360;
////        } else if (deltaAngle > 180) {
////            deltaAngle -= 360;
////        }
////
////        globalAngle += deltaAngle;
////
////        lastAngles = angles;
////
////        return globalAngle;
//   // }
//
//    /**
//     * See if we are moving in a straight line and if not return a power correction value.
//     * @return Power adjustment, + is adjust left - is adjust right.
//     */
//    private double checkDirection()
//    {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = .10;
//
//        angle = getAngle();
//
//        if (angle == 0) {
//            correction = 0;             // no adjustment.
//        } else {
//            correction = -angle;        // reverse sign of angle for correction.
//        }
//
//        correction = correction * gain;
//
//        return correction;
//    }
//
//    /**
//     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
//     * @param degrees Degrees to turn, + is left - is right
//     */
////    private void rotate(int degrees, double power) {
////        double  leftPower, rightPower;
////
////        // restart imu movement tracking.
////        resetAngle();
////
////        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
////        // clockwise (right).
////
////        if (degrees < 0)
////        {   // turn right.
////            leftPower = power;
////            rightPower = -power;
////        }
////        else if (degrees > 0)
////        {   // turn left.
////            leftPower = -power;
////            rightPower = power;
////        }
////        else return;
////
////        // set power to rotate.
////        fLeft.setPower(leftPower);
////        fRight.setPower(rightPower);
////
////        // rotate until turn is completed.
////        if (degrees < 0)
////        {
////            // On right turn we have to get off zero first.
////            while (opModeIsActive() && getAngle() == 0) {}
////
////            while (opModeIsActive() && getAngle() > degrees) {}
////        }
////        else    // left turn.
////            while (opModeIsActive() && getAngle() < degrees) {}
////
////        // turn the motors off.
////        fRight.setPower(0);
////        fLeft.setPower(0);
////
////        // wait for rotation to stop.
////        sleep(1000);
////
////        // reset angle tracking on new heading.
////        resetAngle();
////    }
//}
