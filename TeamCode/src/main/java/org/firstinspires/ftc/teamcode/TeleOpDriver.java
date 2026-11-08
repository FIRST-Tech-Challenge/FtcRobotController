package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Field-Centric Mecanum Drive")

public class TeleOpDriver extends OpMode {
    GoBildaPinpointDriver odo;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor FrontLeft;
    public DcMotor FrontRight;

    @Override
    public void init() {
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

    public void GASGASGAS() {

        double forward = -gamepad1.left_stick_y; // Inverted Y-axis
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        FrontLeft.setPower(newWheelSpeeds[0]);
        FrontRight.setPower(newWheelSpeeds[1]);
        BackLeft.setPower(newWheelSpeeds[2]);
        BackRight.setPower(newWheelSpeeds[3]);

        //More Telemetry
        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed: ", globalForward);
        telemetry.addData("Strafe Speed: ", globalStrafe);









    }

    @Override
    public void loop() {
        GASGASGAS();

        Pose2D pos = odo.getPosition();
        telemetry.addData("RobotX: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("RobotY: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.update();

        odo.update();
    }
}
