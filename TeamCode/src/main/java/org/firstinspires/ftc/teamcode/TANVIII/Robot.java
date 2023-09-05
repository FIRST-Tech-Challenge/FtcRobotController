package org.firstinspires.ftc.teamcode.TANVIII;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

public class Robot {
    HardwareMap hwMap;

    //declare local variables for all motors
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor armMotor;

    public Robot (HardwareMap hardwareMap) {
        this.hwMap = hardwareMap;

        //initialize all motors
        this.fl = hardwareMap.dcMotor.get("fl");
        this.fr = hardwareMap.dcMotor.get("fr");
        this.bl = hardwareMap.dcMotor.get("bl");
        this.br = hardwareMap.dcMotor.get("br");

        this.armMotor = hardwareMap.dcMotor.get("a");

        //reset encoder
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        //arm
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */

        //zero pwr behavior (auto)
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setDrivetrainPower(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    public void setArmPower (double armPower) {
        armMotor.setPower(armPower);
    }

    //getflcurrentpos(name string)

    public int getEncoderPosition (String motorName) {
        if (Objects.equals(motorName, "fl")) {
            return fl.getCurrentPosition();
        } else if (Objects.equals(motorName, "fr")) {
            return fr.getCurrentPosition();
        } else if (Objects.equals(motorName, "bl")) {
            return bl.getCurrentPosition();
        } else if (Objects.equals(motorName, "br")) {
            return br.getCurrentPosition();
        } else {
            assert false;
            return 0;
        }
    }
}
