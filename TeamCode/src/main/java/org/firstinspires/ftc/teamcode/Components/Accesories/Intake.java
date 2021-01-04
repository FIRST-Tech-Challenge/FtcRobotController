/**
 * This class includes all the necessary functions
 * for the intake to work in both Autonomous & Teleop.
 * For example starting the motor to intake rings and
 * stopping the motor.
 *
 * @author Aiden Ma
 * @version 1.0
 * @date 11.10.20
 */

package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor = null;

    final private double intakeSpeed = 1.0;

    // initialization of intakeMotor
    public Intake(LinearOpMode opMode){
        intakeMotor = opMode.hardwareMap.dcMotor.get("IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startIntake() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(intakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
