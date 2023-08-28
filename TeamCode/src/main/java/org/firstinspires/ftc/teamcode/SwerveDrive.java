
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TwoWheelDiffSwerveClass.TICKS_PER_INCH;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This OpMode executes a Swerve Drive control TeleOp for a two wheel/pod differential swerve drive robot
 * The code is structured as an Iterative OpMode
 * In this mode, the Right joysticks controls the direction of motion and speed, while the robot maintains orientation
 * and the left joystick-x changes the orientation (rotation) of the robot
 */
@TeleOp
public class SwerveDrive extends OpMode{
    // Constants
    static final double INITIAL_WHEEL_ANGLE = (Math.PI/2.0); // Wheel angle in radians

    /* Declare OpMode members. */
    double priorTime = 0.0; // for logging loop times

    // IMU (inertia measurement unit, SDK 8.1 and later method of using IMU
    IMU imu;

    // Potentiometer Class
    TwoFullRotationPotClass pots;

    // Drive Class
    TwoWheelDiffSwerveClass drive;

    Orientation or; // robots orientation (x,y,z angles) from IMU on Hub
    double globalIMUHeading;

    double targetPodAngles = INITIAL_WHEEL_ANGLE;
    int deltaS = 0;

    /*
     * Code to run ONCE when the driver touches INIT
     */
    @Override
    public void init() {
        drive = new TwoWheelDiffSwerveClass();
        drive.initDrive(hardwareMap);

        pots = new TwoFullRotationPotClass();
        pots.initPots(hardwareMap);
        pots.getAngleFromPots(false,0); // find out where the wheels are pointed

        drive.initWheelAngles(pots.angle1, pots.angle2,INITIAL_WHEEL_ANGLE,INITIAL_WHEEL_ANGLE);  // set the wheels to desired angle

        // map IMU
        imu = hardwareMap.get(IMU.class,"imu");
        /* The next THREE lines define Hub orientation.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // This seems to help smooth the imu data
        imu.resetDeviceConfigurationForOpMode();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.secondAngle; // save the starting heading

        // Send telemetry message to signify robot waiting;
        telemetry.addData("INITIAL POT 1 ="," %.05f, POT 2 = %.05f",pots.angle1,pots.angle2);
        telemetry.addData(">", "Swerve Robot Ready.  Press Play.");

        // Write to log file
        RobotLog.d("SRA-start-SWERVE DRIVE = %.05f",getRuntime());
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        drive.setMotorsPower(0.8); // full speed  = 1.0. Backed off to prevent damage while developing
    }

    @Override
    public void loop() {
        double gpRightX, gpRightY, gpLeftX;
        double loopTime;
        double fwdSpeed;
        boolean goHeadingUpdate;

        gpRightX = gamepad1.right_stick_x; // used for wheel direction
        gpRightY = gamepad1.right_stick_y; // used for wheel direction
        gpLeftX = gamepad1.left_stick_x; // used for robot orientation

        fwdSpeed = Math.hypot(gpRightX,gpRightY); // used for robot speed

        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        // setup so thet wheels hold current angle if there is no input
        if (fwdSpeed>0.1) { // detect Right joystick movement, update angle
            targetPodAngles = Math.atan2(-gpRightY,-gpRightX); // atan2(y,x) = radians
            if (fwdSpeed>0.2) {
                deltaS = (int) (fwdSpeed*TICKS_PER_INCH);  // amount to move
            } else {
                deltaS = 0;
            }
        } else {
            deltaS = 0;
        }

        // check if the wheel angle supports a heading change
        goHeadingUpdate = ((Math.abs(targetPodAngles)>= Math.PI/2.5) && (Math.abs(targetPodAngles)<=Math.PI/1.5));
        if(goHeadingUpdate) {
            drive.robotAngle += (gpLeftX/10.0); // update the robot angle in radians
        }

        drive.setRobotTranslation(deltaS);
        drive.setMotorPositions(targetPodAngles,targetPodAngles,0.0); // Moves the robot

        telemetry.addData("Right stick y", gpRightY);
        telemetry.addData("Right stick x", gpRightX);
        telemetry.addData("Left stick x", gpLeftX);
        //telemetry.addData("Right Trigger =",fwdSpeed);
        telemetry.addData("Target Pod Angle =","%.03f", targetPodAngles);
        telemetry.addData("Robot Angle =","%.03f", drive.robotAngle);
        telemetry.update();

        loopTime = getRuntime() - priorTime;
        priorTime = getRuntime();

        RobotLog.d("SRA-loop-time = %.05f, loop = %.05f",priorTime,loopTime);
        RobotLog.d("SRA-loop-gamepad = %.03f, %.03f",gamepad1.right_stick_x,gamepad1.right_stick_y);
        RobotLog.d("SRA-IMU Angles RobAngle = %.03f, %.03f, %.03f, %.03f",drive.robotAngle,or.firstAngle,or.secondAngle,or.thirdAngle);
    }

    @Override
    public void stop() {
        RobotLog.d("SRA-end-SWERVE DRIVE = %.05f",getRuntime());
    }
}
