package org.firstinspires.ftc.teamcode.Mechanism.Intake;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.BaseClasses.EctoMechanism;
import org.firstinspires.ftc.teamcode.Core.BaseClasses.OperationModes.EctoOpMode;

public class Intake extends EctoMechanism {

    public Intake(String moduleName, String moduleType, IntakeConfig config) {
        super(moduleName, moduleType);
        this.config = config;
    }
    IntakeConfig config;

    Motor leftIntakeMotor;
    Motor rightIntakeMotor;

    ServoEx leftServo;
    ServoEx rightServo;

    public void intakeEat(double powerPercentage){
        leftIntakeMotor.set(powerPercentage);
        rightIntakeMotor.set(powerPercentage);
    }

    public void intakeBarf(double negativePowerPercentage){
        leftIntakeMotor.set(negativePowerPercentage);
        rightIntakeMotor.set(negativePowerPercentage);
    }

    public void intakeRetracted(double degrees){
        leftServo.turnToAngle(degrees);
        rightServo.turnToAngle(degrees);
    }

    public void intakeExtended(double negativeDegrees){
        leftServo.turnToAngle(negativeDegrees);
        rightServo.turnToAngle(negativeDegrees);
    }

    public void intakeStops(){
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }


    @Override
    public void initMechanism() {
        leftServo = new SimpleServo(
                hardwareMap,
                config.getLeftServoId,
                config.getMinServoAngle,
                config.getMaxServoAngle,
                AngleUnit.DEGREES);

        rightServo =
                new SimpleServo(
                        hardwareMap,
                        config.getRightServoId,
                        config.getMinServoAngle,
                        config.getMaxServoAngle,
                        AngleUnit.DEGREES);

        leftIntakeMotor = new Motor(hardwareMap, config.getLeftServoId, config.getGobildaType);
        leftIntakeMotor.setRunMode(Motor.RunMode.RawPower);

        rightIntakeMotor = new Motor(hardwareMap, config.getRightServoId, config.getGobildaType);
        rightIntakeMotor.setRunMode(Motor.RunMode.RawPower);
    }


    @Override
    public void startMechanism() {

    }

    @Override
    public void updateMechanism() {

    }

    @Override
    public void stopMechanism() {
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }
}
