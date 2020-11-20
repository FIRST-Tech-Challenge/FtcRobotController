package org.firstinspires.ftc.teamcode.backend.control.higher_level;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.middleend.HardwareMappings.HMap;
import org.firstinspires.ftc.teamcode.backend.control.low_level.PIDClass;


public abstract class DistancePID extends LinearOpMode {
    // We know we are working with encoders, so the following methods are to be used for calculating
    // distance traversed

    private HMap robot = new HMap();
    private double wheel_rad = 0.0508; // in m

    // 2.7
    private PIDClass pidClass = new PIDClass(2.7, 0.0000006, 0.0, PIDClass.ERROR_TYPES.RUNNING_WITH_REL_ERROR, telemetry) {
        @Override
        public double getInput() {
            return 0;
        }

        @Override
        public double getTimeElapsed() {
            return robot.runtime.milliseconds();
        }

        @Override
        public void doThingWithResponse(double response) {
            setMotorPower(response);
        }

        @Override
        public boolean opModeActice() {
            return opModeIsActive();
        }
    };
    // Encoder Methods
    private int getEncoderCountFromMotor(DcMotor motor){
        return motor.getCurrentPosition();
    }

//    private int getAvgEncoderPos(){
////        int TL_pos = getEncoderCountFromMotor(robot.TL);
////        int TR_pos = getEncoderCountFromMotor(robot.TR);
////        int BL_pos = getEncoderCountFromMotor(robot.BL);
////        int BR_pos = getEncoderCountFromMotor(robot.BR);
//
////        return (int)((double)(TL_pos + TR_pos + BL_pos + BR_pos) / (double)4); // Not sure whether this type casting is required
//    }

    // Motor Manipulation Methods
    private void setMotorPower(double power){
        robot.TL.setPower(power);
        robot.TR.setPower(power);
        robot.BL.setPower(power);
        robot.BR.setPower(power);
    }

    public void setPIDTarget(double x){
        // This takes a distance in meters, but it should be converted to encoders
        // distance/circumference of wheel = # of rev; 1 rev is 280 encoder ticks
        int encoder_dist = (int)((double)(x/(2*3.14*wheel_rad)) * 1120);
        pidClass.setTarget(encoder_dist);
    }

    public void startPID(){
        robot.init(hardwareMap);
        while (opModeIsActive() && !isStopRequested()){
            pidClass.setPID_LOOP_ACTIVE(true);
            pidClass.PIDLoop();
        }
    }


}
