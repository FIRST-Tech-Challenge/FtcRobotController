package org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.RotationPID.*;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.RotationPID.*;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.FeedForwardConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BT.BTHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.utils.RunCommand;
import org.firstinspires.ftc.teamcode.utils.geometry.BTRotation2d;
import org.firstinspires.ftc.teamcode.utils.geometry.BTTranslation2d;

import java.util.function.DoubleSupplier;

public class ChassisSubsystem extends SubsystemBase {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public ProfiledPIDController m_pidY;
    public ProfiledPIDController m_pidX;
    public ProfiledPIDController m_rotationpid;
    public MotorEx motor_FL;
    public MotorEx motor_FR;
    public MotorEx motor_BL;
    public MotorEx motor_BR;
    public RevIMU gyro;
    private HardwareMap map;
    private VoltageSensor voltageSensor;
    public boolean isFirstTime = true;
    public ElapsedTime driveTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Config
    public static class SpeedsAndAcc {
        public static double maxVelocityX = 0;
        public static double maxVelocityY = 0;
        public static double maxVelocityTheta = 0;
        public static double maxAccelerationX = 0;
        public static double maxAccelerationTheta = 0;
        public static double maxAccelerationY = 0;
    }


    public ChassisSubsystem(HardwareMap map){
        motor_FL = new MotorEx(map, "motor_FL");
        motor_FR = new MotorEx(map, "motor_FR");
        motor_BL = new MotorEx(map, "motor_BL");
        motor_BR = new MotorEx(map, "motor_BR");
        gyro = new RevIMU(map,"imu");
        voltageSensor = map.voltageSensor.iterator().next();

        m_pidX = new ProfiledPIDController(Xkp,Xki,Xkd,new TrapezoidProfile.Constraints(SpeedsAndAcc.maxVelocityX,SpeedsAndAcc.maxAccelerationX));
        m_pidY = new ProfiledPIDController(Ykp,Yki,Ykd, new TrapezoidProfile.Constraints(SpeedsAndAcc.maxVelocityY,SpeedsAndAcc.maxAccelerationY));
        m_rotationpid = new ProfiledPIDController( rkp,rki,rkd,new TrapezoidProfile.Constraints(180,180));

        m_rotationpid.enableContinuousInput(-180,180);
        m_rotationpid.setTolerance(tolerance);

        motor_FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motor_BR.setInverted(true);
        motor_BL.setInverted(false);
        motor_FL.setInverted(false);
        motor_FR.setInverted(true);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro = new RevIMU(map, "imu");
        gyro.init(parameters);

        updateTelemetry();

    }
    public double getAcc(){
        return gyro.getRevIMU().getAcceleration().xAccel;
    }

    public void setMotors(double FL, double FR, double BL, double BR) {
        double compensation = 12.0 / voltageSensor.getVoltage();
        motor_FR.set(compensation * applyFeedForward(ks, kv, FR));
        motor_FL.set(compensation * applyFeedForward(ks, kv, FL));
        motor_BR.set(compensation * applyFeedForward(ks, kv, BR));
        motor_BL.set(compensation * applyFeedForward(ks, kv, BL));
        dashboardTelemetry.addData("compensation", compensation);
    }
    public double applyFeedForward(double ks, double kv, double velocity){

        double s=velocity<0.01? 0: ks*Math.signum(velocity);// this is from kookybotz
        return s + kv * velocity;
    }
    private void drive(double frontVel, double sidewayVel, double rotation) {
        double leftFrontPower  = frontVel + sidewayVel + rotation;
        double rightFrontPower = frontVel - sidewayVel - rotation;
        double leftBackPower   = frontVel - sidewayVel + rotation;
        double rightBackPower  = frontVel + sidewayVel - rotation;
        setMotors(leftFrontPower, rightFrontPower, leftBackPower,rightBackPower);
    }
    public BTCommand fieldRelativeDrive(double frontVel, double sidewayVel, double rotation) {
        return new RunCommand(() -> {

            BTTranslation2d vector = new BTTranslation2d(sidewayVel, frontVel);
            BTTranslation2d rotated = vector.rotateBy(BTRotation2d.fromDegrees(gyro.getHeading()));
            drive(rotated.getY(), rotated.getX(),  rotation);
        }, this);
    }



    @Override
    public void periodic(){
//        motor_BL.set(bl);
//        motor_FL.set(bl);
//        motor_BR.set(bl);
//        motor_FR.set(bl);

        m_rotationpid.setPID(rkp,rki,rkd);
        m_pidY.setPID(Ykp,Yki,Ykd);
        m_pidY.setIntegratorRange(-yMaxIntegral,yMaxIntegral);
        m_rotationpid.setTolerance(tolerance);
        m_pidY.setIzone(YiZone);
        m_pidY.setTolerance(ytolerance);
        m_pidX.setIzone(XiZone);
        m_pidY.setTolerance(xtolerance);
        m_rotationpid.setIzone(rotIzone);
//        odometry.updatePose();//todo: uncomment when starting to use odometry
        m_pidX.setPID(Xkp,Xki,Xkd);
        m_pidX.setConstraints(new TrapezoidProfile.Constraints(SpeedsAndAcc.maxVelocityX,SpeedsAndAcc.maxAccelerationX));

        updateTelemetry();
        updateValues();
    }

    private void updateTelemetry() {
        dashboardTelemetry.addData("gyro", gyro.getAbsoluteHeading());
        dashboardTelemetry.addData("_acc", gyro.getRevIMU().getAcceleration().xAccel);
    }

    private void updateValues() {

    }

    public BTCommand drive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier retaliation) {
        return new RunCommand(() -> {
            if (isFirstTime) {
                driveTimer.reset();
                isFirstTime = false;
            }
            dashboardTelemetry.addData("drive: ", driveTimer.time());
            drive(frontVel.getAsDouble(), sidewayVel.getAsDouble(), retaliation.getAsDouble());
        }, this);

    }

    public BTCommand fieldRelativeDrive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier rotation) {
        return new RunCommand(() -> {

            BTTranslation2d vector = new BTTranslation2d(sidewayVel.getAsDouble(), frontVel.getAsDouble());
            BTTranslation2d rotated = vector.rotateBy(BTRotation2d.fromDegrees(-gyro.getHeading()));
            if(rotated.getNorm()>0.1||Math.abs(rotation.getAsDouble())>0.1) {
                {
                    drive(-rotated.getY() * slowDriver, -rotated.getX() * slowDriver, rotation.getAsDouble() * slowDriver);
                }
            }
            else{drive(0,0,0);}
        }, this);
    }


    public Command stopMotor() {
        return new InstantCommand(()->{
            setMotors(0,0,0,0);
        });
    }

    public Command slowDriving(double multiplier){
        return new InstantCommand(()->slowDriver = multiplier);
    }
    public Command stopSlowDriving(){
        return new InstantCommand(()->slowDriver = 0.8);
    }
}