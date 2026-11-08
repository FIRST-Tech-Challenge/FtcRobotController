package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class TeleOpDriver extends OpMode {
    GoBildaPinpointDriver odo;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor FrontLeft;
    public DcMotor FrontRight;

    public void intialiseDriver() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        BackLeft = hardwareMap.get(DcMotor.class,"BackLeftMotor");
        BackRight= hardwareMap.get(DcMotor.class,"BackRightMotor");
        FrontLeft = hardwareMap.get(DcMotor.class,"FrontLeftMotor");
        FrontRight = hardwareMap.get(DcMotor.class,"FrontRightMotor");

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Odometry Computer Config
        odo.setOffsets(-84.0,-168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Odometry starting pos
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM,-923.925,1601.47, AngleUnit.RADIANS,0);
        odo.setPosition(startingPosition);

        // Telemetry
        telemetry.addData("Status", "Intialised");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y Offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }

    @Override
    public void init() {
    }

    public void GASGASGAS(double speedMultiplier) {
        double forward = -gamepad1.left_stick_y; // Inverted Y-axis
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos(-heading);
        double sinAngle = Math.sin(-heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        double maxPower = 1.0;
        for (double wheelSpeed : newWheelSpeeds) {
            maxPower = Math.max(maxPower, Math.abs(wheelSpeed));
        }
        FrontLeft.setPower((newWheelSpeeds[0] / maxPower) * speedMultiplier);
        FrontRight.setPower((newWheelSpeeds[1] / maxPower) * speedMultiplier);
        BackLeft.setPower((newWheelSpeeds[2] / maxPower) * speedMultiplier);
        BackRight.setPower((newWheelSpeeds[3] / maxPower) * speedMultiplier);

        //More Telemetry
        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed: ", globalForward);
        telemetry.addData("Strafe Speed: ", globalStrafe);
    }

    public void resetHeading() {
        odo.resetPosAndIMU();
    }

    public void DriveLoop(double speedMultiplier) {
        odo.update();
        GASGASGAS(speedMultiplier);

        telemetry.addData("RobotX: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("RobotY: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
