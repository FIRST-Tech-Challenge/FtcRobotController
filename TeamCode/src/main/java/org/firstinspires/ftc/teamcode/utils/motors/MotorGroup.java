package org.firstinspires.ftc.teamcode.utils.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorGroup {

    private final Motor[] MOTORS;
    private final Telemetry TELEMETRY;
    private final ElapsedTime TIME = new ElapsedTime();
    private final int TARGET_POSITION;
    
    public MotorGroup(Motor[] motors, Telemetry telemetry) {
        MOTORS = motors;
        TELEMETRY = telemetry;
        TARGET_POSITION = MOTORS[0].getMotor().getTargetPosition();
    }

    public void move(double distance, double speed, double timeout) {
        for(Motor motor : MOTORS) {
            DcMotor dcMotor = motor.getMotor();
            dcMotor.setTargetPosition(dcMotor.getCurrentPosition() + (int)(distance * motor.getCountsPerInch() * motor.getOffset()));
            // Turn On RUN_TO_POSITION
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // reset the timeout time and start motion.
        TIME.reset();
        for(Motor motor : MOTORS) {
            DcMotor dcMotor = motor.getMotor();
            dcMotor.setPower(Math.abs(speed));
        }
        while(TIME.seconds() < timeout && motorsAreBusy(MOTORS)) {
            // Display it for the driver.
            TELEMETRY.addData("Target",  "Running to ", TARGET_POSITION);
            TELEMETRY.addData("Location",  "Running at ", TARGET_POSITION);
            TELEMETRY.update();
        }
        for(Motor motor : MOTORS) {
            DcMotor dcMotor = motor.getMotor();
            dcMotor.setPower(0);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        TELEMETRY.addData("Run complete and motor ready for next run",  "Position is ", MOTORS[0].getMotor().getCurrentPosition());
        TELEMETRY.update();
    }

    private boolean motorsAreBusy(Motor[] motors) {
        boolean busy = false;
        for(Motor motor : motors) {
            if(!busy && motor.getMotor().isBusy()) {
                busy = true;
            }
        }
        return busy;
    }
    
    

}
