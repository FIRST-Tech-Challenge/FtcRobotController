package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="EchoShooterAuto")
public class EchoShooterAuto extends LinearOpMode {
    private Motor driveLeft, driveRight;
    private FlyWheel flywheel;
    private Hitter hitter;
    private Motor intake1, intake2;
    private BNO055IMU imu;

    private ElapsedTime elapsedTime;
    private Orientation lastAngles;
    private double currentHeading = 0;
    private PIDController pidRotate;

    private double driveSpeed = 0.4;


    public void initRobot() {

        elapsedTime = new ElapsedTime();

        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveRight.setInverted(true);
        driveLeft.setRunMode(Motor.RunMode.RawPower);
        driveRight.setRunMode(Motor.RunMode.RawPower);

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE));
        hitter = new Hitter(hardwareMap.servo.get("sv"));

        pidRotate = new PIDController(Vals.rotate_kp, Vals.rotate_ki, Vals.rotate_kd);
        pidRotate.setTolerance(Vals.rotate_tolerance);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();
    }
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        elapsedTime.reset();

        while(elapsedTime.seconds() < 1.82) {
            double rotatePower = rotate(0);
            double leftPower = -rotatePower;
            double rightPower = rotatePower;

            leftPower += driveSpeed;
            rightPower += driveSpeed;

            driveLeft.set(leftPower);
            driveRight.set(rightPower);
        }
        while (elapsedTime.seconds() < 2) {
            driveLeft.set(0);
            driveRight.set(0);
        }
        int tick = 0;
        while (tick < 3 && elapsedTime.seconds() < 17) {
            flywheel.on();
            if(flywheel.isReady()) {
                hitter.hitFullMotion(0.71);
                tick++;
            }
        }
        elapsedTime.reset();
        while (elapsedTime.seconds() < 1) {
            driveLeft.set(0);
            driveRight.set(0);
        }
        while (elapsedTime.seconds() < 1.8) {
            double rotatePower = rotate(0);
            double leftPower = -rotatePower;
            double rightPower = rotatePower;

            leftPower += driveSpeed;
            rightPower += driveSpeed;

            driveLeft.set(leftPower);
            driveRight.set(rightPower);
        }
        while ( elapsedTime.seconds() < 2) {
            flywheel.off();
        }
        while (elapsedTime.seconds() < 3) {
            driveLeft.set(0);
            driveRight.set(0);
        }
        while (elapsedTime.seconds() < 3.4) {
            double rotatePower = rotate(0);
            double leftPower = -rotatePower;
            double rightPower = rotatePower;

            leftPower -= driveSpeed;
            rightPower -= driveSpeed;

            driveLeft.set(leftPower);
            driveRight.set(rightPower);
        }
        while(elapsedTime.seconds() < 4) {
            driveLeft.set(0);
            driveRight.set(0);
        }
        stop();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = 0;
        pidRotate.reset();
    }

    private double updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if(deltaAngle < -180) {
            deltaAngle += 360;
        } else if(deltaAngle > 180) {
            deltaAngle -= 360;
        }

        currentHeading += deltaAngle;
        lastAngles = angles;

        return currentHeading;
    }

    private double rotate(double degrees) {
        if(Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        double power = pidRotate.calculate(updateHeading(), degrees);
        return power;

    }
}
