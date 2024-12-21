
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaDriveToPoint;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

@TeleOp(name = "Linear OpMode", group = "Techtonics")
public class TT_LinearOpMode extends LinearOpMode {
    private TT_RobotHardware robot = new TT_RobotHardware(this);
    static final Pose2D START_POSITION = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                robot.odo.update();
                robot.nav.driveTo(robot.odo.getPosition(),START_POSITION,0.7,3);
                robot.setDrivePower(robot.nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.LEFT_FRONT), robot.nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.RIGHT_FRONT),
                        robot.nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.LEFT_BACK), robot.nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.RIGHT_BACK));
            } else {
                robot.driveRobot();
                robot.drivelift();
                robot.moveLiftArm();
                robot.moveExtension();
                robot.moveExtensionArm();
                robot.moveExtensionSpin();
            }
            robot.displayTelemetry();
        }
    }
}
