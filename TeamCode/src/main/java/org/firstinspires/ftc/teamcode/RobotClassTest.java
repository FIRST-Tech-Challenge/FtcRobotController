package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.libraries.AutoRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous
public class RobotClassTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AutoRobot robot = new AutoRobot(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            robot.getImu().resetYaw(); //if you move the robot at all between init and running
            //yaw will be incorrect, this handles that
            telemetry.addData("Status", "Running");
            telemetry.update();

            robot.driveForwardsInches(12);
            robot.driveForwardsInches(6);
            robot.driveForwardsInches(3);
            robot.driveForwardsInches(2);
            robot.driveForwardsInches(1);
            robot.waitSeconds(2);
            robot.driveBackwardsInches(24);
            robot.waitSeconds(5);

            robot.driveForwardsInches(36);
            robot.waitSeconds(2);
            robot.driveBackwardsInches(36);
            robot.waitSeconds(5);


            //make sure forwardArcs are precise
            robot.driveArcLeftForwards(180);
            robot.waitSeconds(1);
            telemetry.addData("target", 180);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.driveArcLeftForwards(90);
            robot.waitSeconds(1);
            telemetry.addData("target", 90);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.driveArcLeftForwards(60);
            robot.waitSeconds(1);
            telemetry.addData("target", 30);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.driveArcLeftForwards(30);
            robot.waitSeconds(1);
            telemetry.addData("target", 0);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.driveArcRightForwards(180);
            robot.waitSeconds(1);
            telemetry.addData("target", 180);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.driveArcRightForwards(90);
            robot.waitSeconds(1);
            telemetry.addData("target", -90);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.driveArcRightForwards(60);
            robot.waitSeconds(1);
            telemetry.addData("target", -30);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);
            robot.driveArcRightForwards(30);
            robot.waitSeconds(1);
            telemetry.addData("target", 0);
            telemetry.addData("actual", robot.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            robot.waitSeconds(1);


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
            robot.driveForwardsSeconds(1);
            robot.face(-90);
            robot.driveForwardsSeconds(1);
            robot.face(180);
            robot.driveForwardsSeconds(1);
            robot.face(90);
            robot.driveForwardsSeconds(1);
            robot.face(0);
            //drive right in a square
            robot.driveRightSeconds(1);
            robot.face(90);
            robot.driveRightSeconds(1);
            robot.face(180);
            robot.driveRightSeconds(1);
            robot.face(-90);
            robot.driveRightSeconds(1);
            robot.face(0);

            //drive backwards in a square
            robot.face(180);
            robot.driveBackwardsSeconds(1);
            robot.face(90);
            robot.driveBackwardsSeconds(1);
            robot.face(0);
            robot.driveBackwardsSeconds(1);
            robot.face(-90);
            robot.driveBackwardsSeconds(1);

            //drive in a square without turning
            robot.face(0);
            robot.driveForwardsSeconds(1);
            robot.driveRightSeconds(1.0);
            robot.driveBackwardsSeconds(1);
            robot.driveLeftSeconds(1);







        }



    }
}
