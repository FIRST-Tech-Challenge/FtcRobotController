package org.firstinspires.ftc.teamcode.Mechanism.Intake;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeConfig {
    public IntakeConfig(String leftIntakeMotorId,String rightIntakeMotor, String leftServoId, String rightServoId, int minServoAngle, int maxServoAngle, Motor.GoBILDA gobildaMotorType){
        getLeftIntakeMotorId = leftIntakeMotorId;
        getRightIntakeMotorId = rightIntakeMotor;
        getGobildaType = gobildaMotorType;
        getLeftServoId = leftServoId;
        getRightServoId = rightServoId;
        getMaxServoAngle = maxServoAngle;
        getMinServoAngle = minServoAngle;
    }
    public String getLeftServoId;
    public String getRightServoId;
    public String getLeftIntakeMotorId;
    public String getRightIntakeMotorId;
    public Motor.GoBILDA getGobildaType;

    int getMinServoAngle;
    int getMaxServoAngle;
}
