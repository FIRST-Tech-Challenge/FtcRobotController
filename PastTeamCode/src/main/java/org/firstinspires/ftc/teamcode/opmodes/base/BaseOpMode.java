package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.ImuSystem;
import org.firstinspires.ftc.teamcode.components.IntakeSystem;
import org.firstinspires.ftc.teamcode.components.TurnTableSystem;
import org.firstinspires.ftc.teamcode.helpers.Constants;
import org.firstinspires.ftc.teamcode.helpers.GameState;

import java.util.EnumMap;
import java.util.Map;

// TODO - Finalize INTAKESYSTEM TO WORK PROPERLY TOGETHER EVERYWHERE
public abstract class BaseOpMode extends OpMode {

    protected ArmSystem armSystem;
    protected IntakeSystem intakeSystem;
    protected TurnTableSystem turnTableSystem;
    protected ImuSystem imuSystem;
    protected ElapsedTime elapsedTime;
    protected DriveSystem driveSystem;

    @Override
    public void init() {
        imuSystem = new ImuSystem(hardwareMap.get(BNO055IMU.class, Constants.IMU));
        Map<DriveSystem.MotorNames, DcMotor> motors = new EnumMap<>(DriveSystem.MotorNames.class);
        motors.put(DriveSystem.MotorNames.FRONTRIGHT, hardwareMap.get(DcMotor.class, Constants.MOTOR_FRONT_RIGHT));
        motors.put(DriveSystem.MotorNames.FRONTLEFT, hardwareMap.get(DcMotor.class, Constants.MOTOR_FRONT_LEFT));
        motors.put(DriveSystem.MotorNames.BACKRIGHT, hardwareMap.get(DcMotor.class, Constants.MOTOR_BACK_RIGHT));
        motors.put(DriveSystem.MotorNames.BACKLEFT, hardwareMap.get(DcMotor.class, Constants.MOTOR_BACK_LEFT));

        armSystem = new ArmSystem(hardwareMap.get(DcMotor.class, Constants.ELEVATOR_MOTOR), hardwareMap.get(AnalogInput.class, "p"));
        driveSystem = new DriveSystem(motors, hardwareMap.get(BNO055IMU.class, Constants.IMU));
        intakeSystem = new IntakeSystem(hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR1), hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR2));
        turnTableSystem = new TurnTableSystem(hardwareMap.get(DcMotor.class, Constants.ROTATOR_MOTOR));elapsedTime = new ElapsedTime();
    }

    @Override
    public void start() {
        super.start();
        armSystem.initMotors();
        intakeSystem.initMotors();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        //armSystem.moveToPosition(ArmSystem.LEVEL_CAROUSEL);
    }


}
