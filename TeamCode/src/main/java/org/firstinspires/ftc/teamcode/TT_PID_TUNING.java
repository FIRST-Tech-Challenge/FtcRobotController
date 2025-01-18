
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaDriveToPoint;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import java.util.Locale;

@TeleOp(name = "PID TUNING", group = "Techtonics")
public class TT_PID_TUNING extends LinearOpMode {
    private TT_RobotHardware robot = new TT_RobotHardware(this);
    static final Pose2D START_POSITION = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    private double autonomousPower = .6;
    Pose2D GOTO = new Pose2D(DistanceUnit.MM, calcXCoordinate(-700), 0, AngleUnit.DEGREES, 0);
    final Pose2D RIGHT = new Pose2D(DistanceUnit.MM, calcXCoordinate(-600), 725, AngleUnit.DEGREES, 0);
    final Pose2D FORWARD = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1250), 725, AngleUnit.DEGREES, 0);
    final Pose2D BACK = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1250), 863, AngleUnit.DEGREES, 0);

    final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, calcXCoordinate(2000), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(0), 0, AngleUnit.DEGREES, 0);

    private double calcXCoordinate(double xChange) {
        return xChange + 0;
    }

    int xCoor = 0;
    int yCoor = 0;

    private static double xyTolerance = 20;
    private static double pGain = 0.008;
    private static double dGain = 0.00001;
    private static double accel = 8.0;

    enum StateMachine {

        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8
    }

    @Override
    public void runOpMode() {
        robot.init();
        StateMachine stateMachine;
        stateMachine = StateMachine.DRIVE_TO_TARGET_1;
        robot.nav.setXYCoefficients(pGain, dGain, accel, MM, xyTolerance);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine) {
                case DRIVE_TO_TARGET_1:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_1, autonomousPower, 2)) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_2, autonomousPower, 2)) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    }
                    break;
            }
            if (gamepad1.dpad_left) {
                autonomousPower -= .01;
            } else if (gamepad1.dpad_right) {
                autonomousPower += .01;
            } else if (gamepad1.a) {
                dGain -= .00001;
            } else if (gamepad1.b) {
                dGain += .00001;
            } else if (gamepad1.x) {
                pGain -= .001;
            } else if (gamepad1.y) {
                pGain += .001;
            }

            robot.nav.setXYCoefficients(pGain, dGain, accel, MM, xyTolerance);
            robot.setDrivePower();
            telemetry.addData("Target", "X:   %4d     Y:   %4d", xCoor, yCoor);
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", "X:   %4d     Y:   %4d", xCoor, yCoor);
            telemetry.addData("P Gain", "%1.3f", pGain);
            telemetry.addData("D Gain", "%1.5f", dGain);
            telemetry.addData("Acceleration", "%1.1f", accel);
            telemetry.addData("Power", "%1.2f", autonomousPower);
            telemetry.update();

        }
    }
}

