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
            robot.getImu().resetYaw(); //if you move the robot at all between init and running it will
            //be incorrect
            telemetry.addData("Status", "Running");
            telemetry.update();

            //make sure robot does not get stuck while turning
            robot.face(10);
            robot.waitSeconds(1);
            telemetry.addData("target", 10);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(-10);
            robot.waitSeconds(1);
            telemetry.addData("target", -10);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(20);
            robot.waitSeconds(1);
            telemetry.addData("target", 20);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(-20);
            robot.waitSeconds(1);
            telemetry.addData("target", -20);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.face(40);
            robot.waitSeconds(1);
            telemetry.addData("target", 40);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(-40);
            robot.waitSeconds(1);
            telemetry.addData("target", -40);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(90);
            robot.waitSeconds(1);
            telemetry.addData("target", 90);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(-90);
            robot.waitSeconds(1);
            telemetry.addData("target", -90);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(180);
            robot.waitSeconds(1);
            telemetry.addData("target", 180);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.face(0);
            robot.waitSeconds(1);
            telemetry.addData("target", 0);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);



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
            robot.driveRight(1.0);
            robot.driveBackward(1);
            robot.driveLeft(1);







        }



    }
}
