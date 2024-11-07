package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import java.util.HashMap;

public class MecanumDriveSubsystem extends SubsystemBase implements IDriveTrainSubsystem {
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private final HashMap<MotorNames, MotorEx> motors = new HashMap<>();

    public MecanumDriveSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Getting motors");
        this.motors.put(MotorNames.FRONT_RIGHT, new MotorEx(hardwareMap, MotorMap.LEG_FRONT_RIGHT.getId()));
        this.motors.put(MotorNames.FRONT_LEFT, new MotorEx(hardwareMap, MotorMap.LEG_FRONT_LEFT.getId()));
        this.motors.put(MotorNames.BACK_LEFT, new MotorEx(hardwareMap, MotorMap.LEG_BACK_LEFT.getId()));
        this.motors.put(MotorNames.BACK_RIGHT, new MotorEx(hardwareMap, MotorMap.LEG_BACK_RIGHT.getId()));

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Inverting motors");
        this.motors.get(MotorNames.FRONT_RIGHT).setInverted(true);
        this.motors.get(MotorNames.BACK_RIGHT).setInverted(true);
    }

    public void moveMotor(MotorNames motor, double power) {
        MotorEx m = this.motors.get(motor);
        if (m == null) {
            this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": failed to set power to the motor '" + motor.name() + "'");
            return;
        }
        m.set(power);
    }

    @Override
    public void moveSideMotors(double left, double right) {
        this.moveMotor(MotorNames.FRONT_LEFT, left);
        this.moveMotor(MotorNames.BACK_LEFT, left);

        this.moveMotor(MotorNames.FRONT_RIGHT, right);
        this.moveMotor(MotorNames.BACK_RIGHT, right);
    }

    @Override
    public double getForwardDistanceDriven() {
        return RobotPositionManager.getInstance().getRightWheelDistanceDriven();
    }

    public double getSideDistanceDriven() {
        return RobotPositionManager.getInstance().getBackWheelDistanceDriven();
    }

    @Override
    public double getHeading() {
        return RobotPositionManager.getInstance().getHeadingByGyro();
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return this.telemetry;
    }

    public enum MotorNames {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
