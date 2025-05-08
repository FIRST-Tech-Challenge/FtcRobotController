package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * Basic driveTrain class that have common functions usable for both auto & teleop
 * <p>The drive train should only hold the hardware related reference and method, all the
 * driving logic will be in drive related command</p>
 */
@Config
public class DriveTrain extends SubsystemBase {

    protected static final double OdometryWheelCircumference = 150.792;
    protected static final double METER_PER_SEC_TO_POWER = 400;

    // default to no power
    protected double powerRatio = 0;

    // 1 means drive forward, -1 means drive backward
    protected double directionFlag = 1;

    protected final GamepadEx gamepad;
    protected MecanumDrive drive;
    protected final Telemetry telemetry;

    protected DriverFeedback feedback;

    protected boolean revertMotor = false;
    protected Motor fL;
    protected Motor fR;
    protected Motor bL;
    protected Motor bR;
    protected DistanceSensor forwardDistanceSensor;
    GoBildaPinpointDriver odo;
    protected LimeLight limeLight;

    boolean showTelemetry = false;

    @Override
    public void periodic() {
        if (showTelemetry) {
            odo.update();

            telemetry.addData("odo x", odo.getPosX());
            telemetry.addData("odo y", odo.getPosY());
            telemetry.addData("odo theta (degree)", Math.toDegrees(odo.getHeading()));
            telemetry.update();
        }
    }

    public DriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, LimeLight limeLight) {
        this(hardwareMap, gamepad, telemetry, feedback, true, limeLight);
    }

    public DriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, boolean revertMotor, LimeLight limeLight) {

        this.revertMotor = revertMotor;
        /* instantiate motors */
        initWheels(hardwareMap);
        createDrive();
        initOdo(hardwareMap);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.limeLight = limeLight;


    }

    private void initOdo(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.resetPosAndIMU();
        odo.setOffsets(Units.inchesToMMs(-6.5), Units.inchesToMMs(5.5));

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    protected void createDrive() {
        this.drive = new MecanumDrive(this.fL, this.fR, this.bL, this.bR);
        if (revertMotor) {
            this.drive.setRightSideInverted(true);
        }
    };

    protected void initWheels(HardwareMap hardwareMap) {
        this.fL = new Motor(hardwareMap, "FL");
        this.bL = new Motor(hardwareMap, "BL");
        this.fR = new Motor(hardwareMap, "FR");
        this.bR = new Motor(hardwareMap, "BR");

        forwardDistanceSensor = hardwareMap.get(DistanceSensor.class, "ForwardDistanceSensor");

        fL.encoder.reset();
        fR.encoder.reset();
        bL.encoder.reset();
        bR.encoder.reset();

        fL.setRunMode(Motor.RunMode.RawPower);
        fR.setRunMode(Motor.RunMode.RawPower);
        bL.setRunMode(Motor.RunMode.RawPower);
        bR.setRunMode(Motor.RunMode.RawPower);

//        fL.setInverted(false);
//        fR.setInverted(true);
//        bL.setInverted(false);
//        bR.setInverted(true);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        if (revertMotor) {
            this.fR.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            this.bR.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    };

    public void stop() {
        drive.stop();
    }

    public void ToggleDriveDirection() {

        directionFlag = directionFlag * -1d;

        if (feedback != null) {
            if (isDrivingForward()) {
                feedback.DriverControllerGreen();
            } else {
                feedback.DriverControllerRed();
            }
            feedback.DriverRumbleBlip(1);
        }
    }

    public void setPowerRatio(double powerRatio) {
        this.powerRatio = powerRatio;
    }

    private boolean isDrivingForward() {
        return directionFlag > 0;
    }

    public double GetForwardDistanceFromObstacleInMM() {
        return this.forwardDistanceSensor.getDistance(DistanceUnit.MM);
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }

    public Motor getfL() {
        return fL;
    }

    public Motor getfR() {
        return fR;
    }

    public Motor getbL() {
        return bL;
    }

    public Motor getbR() {
        return bR;
    }

    public boolean AlignTx() {
        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.2;


        if(result != null) {
            double tx = result.getTx();
            telemetry.addLine("AlignTx");

            telemetry.addData("tx", tx);

            while(Math.abs(tx) > 1.0) {

                double power = tx * -0.01;
                if (Math.abs(power) < min) {
                    power = min * Math.signum(power);
                }
                telemetry.addData("turn power", power);

                drive.driveRobotCentric(0, power, 0);

                result = limeLight.GetResult();

                if (result == null) {
                    break;
                }

                tx = result.getTx();

                telemetry.update();
            }
        }
        else {
            telemetry.addLine("No sample found");
            return false;
        }

        telemetry.update();

        drive.stop();
        return true;
    }

    public void AlignTy() {
        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.2;

        if(result != null) {
            double ty = result.getTy() - 3.0;

            while(Math.abs(ty) > 1) {

                double power = ty * -0.01;
                if(Math.abs(power) < min) {
                    power = min * Math.signum(power);
                }
                drive.driveRobotCentric(0, 0, -1 * power);

                result = limeLight.GetResult();

                if(result == null) {
                    telemetry.addLine("cannot find sample");
                    break;
                }

                ty = result.getTy();
            }

            drive.stop();
        }
    }

    public double GetPosX() {
        odo.update();

        return odo.getPosX();
    }

    public double GetPosY() {
        odo.update();

        return odo.getPosY();
    }

    public void setWheelsPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        fL.motor.setPower(frontLeftPower);
        fR.motor.setPower(frontRightPower);
        bL.motor.setPower(backLeftPower);
        bR.motor.setPower(backRightPower);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

//    /// ---
//    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
//        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
//    }

//    private double GetOdometryDistance() {
//        return OdometryWheelCircumference * this.fL.encoder.getPosition() / 2000;
//    }
//
//    private double GetStrafeDistance() {
//        return OdometryWheelCircumference * this.bL.encoder.getPosition() / 2000;
//    }

}