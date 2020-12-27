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

    //TODO <owner> : Do we need to store the reference to LinearOpMode here? Unless you want to use this to show some telemetry data, we don't need store a reference.
    private LinearOpMode op = null;

    //TODO <owner> : Do we need to store the reference to hardware map here?
    private HardwareMap hardwareMap = null;

    private DcMotor intakeMotor = null;

    final private double intakeSpeed = 1.0;

    // initialization of intakeMotor
    public Intake(LinearOpMode opMode){
        op = opMode;
        hardwareMap = op.hardwareMap;

        intakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
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
