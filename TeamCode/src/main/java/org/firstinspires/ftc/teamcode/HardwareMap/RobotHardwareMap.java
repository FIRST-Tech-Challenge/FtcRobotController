package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Deprecated
public class RobotHardwareMap {
    public DriveTrain driveTrain;

    public DcMotor TL, TR, BL, BR;

    // Why would this class really be important?
    private static class DriveTrain{
        enum WHEEL_CONTROL{
            INDEPENDENT_CONTROL, LEFT_DRIVE_RIGHT_DRIVE_CONTROL
        }
        // Constants
        private int wheel_num;

        //Parameters
        // name of the 4 wheels (can be changed according to num of wheels)
        private String[] motor_config_names;
        private WHEEL_CONTROL selected_wheel_control;

        // This class deals with the drive train abstraction
        public DriveTrain(){
            // Base Constructor (no predef settings)
        }

        public DriveTrain(String[] motor_config_names, WHEEL_CONTROL control_type){
            // Constructor with motor names in TL, TR, BL, BR, format
            this.motor_config_names = motor_config_names;
            this.selected_wheel_control = control_type;
        }

        // Methods for interacting with DriveTrain class
        public String getTLMotor(){
            return motor_config_names[0];
        }

        public String getTRMotor(){
            return motor_config_names[1];
        }

        public String getBLMotor(){
            return motor_config_names[2];
        }

        public String getBRMotor(){
            return motor_config_names[3];
        }
    }
    // Hardware Map Variables
    HardwareMap hwmap = null;

    // DriveTrain Interaction
    public void setDriveTrain(){
        this.driveTrain = new DriveTrain();
    }

    public void setDriveTrain(String[] motor_config_names, DriveTrain.WHEEL_CONTROL control_type){
        this.driveTrain = new DriveTrain(motor_config_names, control_type);
    }

    // Init Funciton
    public void init(HardwareMap ahwmap){
        hwmap = ahwmap;
        TL = hwmap.get(DcMotor.class, driveTrain.getTLMotor());
        TR = hwmap.get(DcMotor.class, driveTrain.getTRMotor());
        BL = hwmap.get(DcMotor.class, driveTrain.getBLMotor());
        BR = hwmap.get(DcMotor.class, driveTrain.getBRMotor());

        // Set zero power
        TL.setPower(0.0);
        BL.setPower(0.0);
        TR.setPower(0.0);
        BR.setPower(0.0);

        TR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Encoder Stuff
        TL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
