package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.HelperClasses.Odometry;
//import org.firstinspires.ftc.teamcode.roadrunnerquickstart.drive.util.Encoder;

@TeleOp(name = "Odom Test",group="teleop")
public class OdomTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Hardware.currentOpMode=this;
        Odometry robot = new Odometry(hardwareMap);
        waitForStart();
        robot.softBrake();
        robot.resetOdometry(0,0,0);

        robot.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        while(opModeIsActive())
        {
            robot.updatePositionRoadRunner();
            robot.robotODrive(0, 0, gamepad1.right_stick_x);
            telemetry.addData("left", robot.leftEncoderPos);
            telemetry.addData("right", robot.rightEncoderPos);
            telemetry.addData("center", robot.centerEncoderPos);
            telemetry.addLine();
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.addLine();
            telemetry.update();
        }
    }
}
