package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Calibrate Intake")
public class CalibrateIntake extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();
        /*
        if(intakeMotor != null) {
            intakeMotor.setPower(INTAKE_SPEED);
            sleep((long) INTAKE_TIME);
            intakeMotor.setPower(0);
        } else {
            sleep(5000);
        }

        sleep(5000);
        */

        if(intakeMotor != null) {
            intakeMotor.setPower(INTAKE_SPEED2);
            sleep((long) INTAKE_TIME2);
            intakeMotor.setPower(0);
        } else {
            sleep(5000);
        }
    }
}
