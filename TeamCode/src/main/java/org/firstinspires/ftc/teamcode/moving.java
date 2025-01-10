package org.firstinspires.ftc.teamcode;

/* System includes */
import java.util.Map;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfImu;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.components.MotorComponent;


public class Moving {

    Telemetry       mLogger;

    boolean         mReady;

    IMU             mImu;

    MotorComponent  mFrontLeftMotor;
    MotorComponent  mBackLeftMotor;
    MotorComponent  mFrontRightMotor;
    MotorComponent  mBackRightMotor;

    Gamepad         mGamepad;

    boolean         mIsFieldCentric = true;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger, Gamepad gp) {

        mLogger = logger;
        mLogger.addLine("===== MOVING =====");

        mReady = true;

        /// Get wheels and IMU parameters from configuration
        ConfMotor frontLeftWheel  = config.getMotor("front-left-wheel");
        ConfMotor frontRightWheel = config.getMotor("front-right-wheel");
        ConfMotor backLeftWheel   = config.getMotor("back-left-wheel");
        ConfMotor backRightWheel  = config.getMotor("back-right-wheel");
        ConfImu imu               = config.getImu("built-in");

        if (mIsFieldCentric) { mLogger.addLine("==>  FIELD CENTRIC"); }
        else                 { mLogger.addLine("==>  ROBOT CENTRIC"); }

        String status = "";
        if(frontLeftWheel == null)         { status += " FL";  mReady = false; }
        if(frontRightWheel == null)        { status += " FR";  mReady = false; }
        if(backLeftWheel == null)          { status += " BL";  mReady = false; }
        if(backRightWheel == null)         { status += " BR";  mReady = false; }
        if(mIsFieldCentric && imu == null) { status += " IMU"; mReady = false; }

        if(mReady) { mLogger.addLine("==>  CONF : OK"); }
        else         { mLogger.addLine("==>  CONF : KO : " + status); }

        if (mReady) {

            status = "";

            mFrontLeftMotor = new MotorSingle(frontLeftWheel, hwm, "front-left-wheel",mLogger);
            mBackLeftMotor = new MotorSingle(backLeftWheel, hwm, "back-left-wheel",mLogger);
            mFrontRightMotor = new MotorSingle(frontRightWheel, hwm, "front-right-wheel",mLogger);
            mBackRightMotor = new MotorSingle(backRightWheel, hwm, "back-right-wheel",mLogger);

            mImu = null;
            if(imu != null) { mImu = hwm.tryGet(IMU.class, imu.getName()); }

            if (!mFrontLeftMotor.isReady())     { status += " FL";  mReady = false; }
            if (!mFrontRightMotor.isReady())    { status += " FR";  mReady = false; }
            if (!mBackLeftMotor.isReady())      { status += " BL";  mReady = false; }
            if (!mBackRightMotor.isReady())     { status += " BR";  mReady = false; }
            if(mIsFieldCentric && mImu == null) { status += " IMU"; mReady = false; }

            if(mReady) { logger.addLine("==>  HW : OK"); }
            else        { logger.addLine("==>  HW : KO : " + status); }

        }

        if(mReady) {

            mFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(mIsFieldCentric) {
                RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                        imu.getLogo(), imu.getUsb());
                mImu.initialize(new IMU.Parameters(RevOrientation));
                mImu.resetYaw();
            }

        }

        mGamepad = gp;

        if(mReady) { logger.addLine("==>  READY"); }
        else       { logger.addLine("==>  NOT READY"); }
    }

    public void move() {

        if (mReady) {

            double multiplier = 0.45;
            if (mGamepad.left_bumper)  { multiplier = 0.9; }
            if (mGamepad.right_bumper) { multiplier = 0.25; }

            double y = -applyDeadzone(mGamepad.left_stick_y, 0.1);
            double x = applyDeadzone(mGamepad.left_stick_x, 0.1) * 1;
            double rotation = applyDeadzone(mGamepad.right_stick_x, 0.1);
            mLogger.addLine(String.format("\n==>  X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

            if (mIsFieldCentric) {
                double heading = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
                mLogger.addLine(String.format("==>  HD %6.1f X : %6.1f Y : %6.1f", heading,x,y));
            }
            x *= 1.1; // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
            double frontLeftPower = (y + x + rotation) / denominator * multiplier;
            double backLeftPower = (y - x + rotation) / denominator * multiplier;
            double frontRightPower = (y - x - rotation) / denominator * multiplier;
            double backRightPower = (y + x - rotation) / denominator * multiplier;

            mFrontLeftMotor.setPower(frontLeftPower);
            mBackLeftMotor.setPower(backLeftPower);
            mFrontRightMotor.setPower(frontRightPower);
            mBackRightMotor.setPower(backRightPower);
        }
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0; // Inside deadzone
        }
        // Scale the value to account for the deadzone
        double scaledValue = (value - Math.signum(value) * deadzone) / (1.0 - deadzone);
        return scaledValue;
    }

}