
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorPair;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;


import java.lang.Thread;
public class ArmExtensionSubsystem extends SubsystemBase {
    // Declare the motor
    private DcMotorEx armExtensionMotor;

    // incase you are confused, these two motors control the angle of the arm. The actions of both of these motors needs to be synced in order to not break something
    private DcMotorEx masterAngleMotor;
    private DcMotorEx slaveAngleMotor;
    private DcMotorPair syncedArmMotors;
    private HardwareMap hardwareMap;
    private final OpMode opMode;
    public ArmExtensionSubsystem(HardwareMap hardwareMap,OpMode opMode) {
        this.opMode = opMode; // i saw bosco doing this so i did it too

        // Initialize hardware
        masterArmAngleMotor = hardwareMap.get(DcMotorEX.class, IntakeConstants.MASTER_ANGLE_MOTOR);
        slaveArmAngleMotor = hardwareMap.get(DcMotorEX.class, IntakeConstants.SLAVE_ANGLE_MOTOR);
        armExtensionMotor = hardwareMap.get(DcMotorEX.class, IntakeConstants.ARM_EXTENSION_MOTOR);

        syncedArmAngleMotors = new DcMotorPair(Constants.MASTER_ANGLE_MOTOR_PORTNAME, SLAVE_ANGLE_MOTOR_PORTNAME);

        // brakes if not is pressed
        horizontalSlideLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }


    // good method, always have this in mechanisms where you need to keep track of its movement. this avoids unpredicatable behavior.
    public void resetEncoders() {
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAngleMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAngleMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
