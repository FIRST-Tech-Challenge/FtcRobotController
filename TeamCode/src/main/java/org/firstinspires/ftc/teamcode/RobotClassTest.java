package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.AutoRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




import org.firstinspires.ftc.teamcode.auto.OdometryMotor;
@Autonomous
public class RobotClassTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AutoRobot robot = new AutoRobot(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();

            //make sure robot does not get stuck while turning
            robot.face(10);
            robot.face(-10);
            robot.face(20);
            robot.face(-20);
            robot.face(40);
            robot.face(-40);
            robot.face(90);
            robot.face(-90);
            robot.face(180);
            robot.face(0);


            //drive forward in a square
            robot.driveForward(1);
            robot.face(-90);
            robot.driveForward(1);
            robot.face(180);
            robot.driveForward(1);
            robot.face(90);
            robot.driveForward(1);
            robot.face(0);
            //drive right in a square
            robot.driveRight(1);
            robot.face(90);
            robot.driveRight(1);
            robot.face(180);
            robot.driveRight(1);
            robot.face(-90);
            robot.driveRight(1);
            robot.face(0);

            //drive backwards in a square
            robot.face(180);
            robot.driveBackward(1);
            robot.face(90);
            robot.driveBackward(1);
            robot.face(0);
            robot.driveBackward(1);
            robot.face(-90);
            robot.driveBackward(1);

            //drive in a square without turning
            robot.face(0);
            robot.driveForward(1);
            robot.driveRight(1);
            robot.driveBackward(1);
            robot.driveLeft(1);







        }



    }
}
