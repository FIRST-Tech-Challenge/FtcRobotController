package org.firstinspires.ftc.teamcode.Team.OpModes;
//Kirill was here :)
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.teamcode.Team.ComplexRobots.Robot;

@TeleOp
public class FieldCentricStrafeTest extends LinearOpMode {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        //Makes a new IMU object
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //Sets IMU Units to Radian
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //Inits IMU
        imu.initialize(parameters);
        //Waits for play button
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //gets stick coordinates
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            //Gets the bots heading (direction)
            double botHeading = -imu.getAngularOrientation().firstAngle;
            //Does trig on it to determine vectors of movement in x and y directions
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            //Adds the X, Y, and rotation vectors in different combination to determine power to each wheel
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            //Sets the motor power
            robot.motors[robot.LFMotor].setPower(frontLeftPower);
            robot.motors[robot.RBMotor].setPower(backRightPower);
            robot.motors[robot.RFMotor].setPower(frontRightPower);
            robot.motors[robot.LBMotor].setPower(backLeftPower);
        }
    }
}