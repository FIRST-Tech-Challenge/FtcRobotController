package org.firstinspires.ftc.team417_CENTERSTAGE.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.League1BaseAutonomous;

@Autonomous(name="Calibrate Intake")
public class CalibrateIntake extends League1BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();

        if(intakeMotor != null) {
            intakeMotor.setPower(INTAKE_SPEED2);
            sleep((long) INTAKE_TIME2);
            intakeMotor.setPower(0);
        } else {
            sleep(5000);
        }
    }
}
