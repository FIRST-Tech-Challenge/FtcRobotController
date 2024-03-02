package org.firstinspires.ftc.teamcode.Testing.Helper_test;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class motorCmds extends LinearOpMode {

    public DcMotor lfDrive = null;  //  Used to control the left front drive wheel
    public DcMotor rfDrive = null;  //  Used to control the right front drive wheel
    public DcMotor lbDrive = null;  //  Used to control the left back drive wheel
    public DcMotor rbDrive = null;  //  Used to control the right back drive wheel

    public void mtrDirection(DcMotor.Direction lfDirection, DcMotor.Direction rfDirection,
                             DcMotor.Direction lbDirection,DcMotor.Direction rbDirection) {
        lfDrive.setDirection(lfDirection);
        rfDrive.setDirection(rfDirection);
        lbDrive.setDirection(lbDirection);
        rbDrive.setDirection(rbDirection);
    }
    public void mapHardware() {
        lfDrive = hardwareMap.get(DcMotor.class, "lf_drive");
        rfDrive = hardwareMap.get(DcMotor.class, "rf_drive");
        lbDrive = hardwareMap.get(DcMotor.class, "lb_drive");
        rbDrive = hardwareMap.get(DcMotor.class, "rb_drive");
    }
    public void pause() {
        lfDrive.setPower(0);
        rfDrive.setPower(0);
        lbDrive.setPower(0);
        rbDrive.setPower(0);
    }

    public void move_forward(double Speed, long Sleep) {
        lfDrive.setPower(Speed);
        rfDrive.setPower(Speed);
        lbDrive.setPower(Speed);
        rbDrive.setPower(Speed);
        sleep(Sleep);
    }

    public void move_backward(double Speed, long Sleep) {
        lfDrive.setPower(-Speed);
        rfDrive.setPower(-Speed);
        lbDrive.setPower(-Speed);
        rbDrive.setPower(-Speed);
        sleep(Sleep);
    }

    public void strafe_left(double Speed, long Sleep) {
        lfDrive.setPower(-Speed);
        rfDrive.setPower(Speed);
        lbDrive.setPower(Speed);
        rbDrive.setPower(-Speed);
        sleep(Sleep);
    }

    public void strafe_right(double Speed, long Sleep) {
        lfDrive.setPower(Speed);
        rfDrive.setPower(-Speed);
        lbDrive.setPower(-Speed);
        rbDrive.setPower(Speed);
        sleep(Sleep);
    }

    public void rotate_left(double Speed, long Sleep) {
        lfDrive.setPower(-Speed);
        rfDrive.setPower(Speed);
        lbDrive.setPower(-Speed);
        rbDrive.setPower(Speed);
        sleep(Sleep);
    }

    public void rotate_right(double Speed, long Sleep) {
        lfDrive.setPower(Speed);
        rfDrive.setPower(-Speed);
        lbDrive.setPower(Speed);
        rbDrive.setPower(-Speed);
        sleep(Sleep);
    }

    @Override
    public void runOpMode() {
    }
}
