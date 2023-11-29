package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoMethods {
    double TicsPerRevolution = 537.6;
    double Circumference = 11.87;
    double HangInchesPerRev = 0.25;
    double TPI = TicsPerRevolution / Circumference;
    double StrafeTPI = 50.2512563;
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorIntake, motorHang;

    public AutoMethods(DcMotor left, DcMotor left2, DcMotor right, DcMotor right2, DcMotor inTake, DcMotor hang) {
        motorLeft = left;
        motorLeft2 = left2;
        motorRight = right;
        motorRight2 = right2;
        motorIntake = inTake;
        motorHang = hang;
    }

    int StrafeInchesToTicks(double inches) {
        return (int) (inches * StrafeTPI);
    }

    void StrafeByInch(double inches, boolean sendright, double motorPower) throws InterruptedException {
        int Ticks = StrafeInchesToTicks(inches);
        if (sendright) {
            motorRight2.setTargetPosition(motorRight2.getCurrentPosition() - Ticks);
            motorRight.setTargetPosition(motorRight.getCurrentPosition() + Ticks);
            motorLeft2.setTargetPosition(motorLeft2.getCurrentPosition() + Ticks);
            motorLeft.setTargetPosition(motorLeft.getCurrentPosition() - Ticks);
        } else {
            motorRight2.setTargetPosition(motorRight2.getCurrentPosition() + Ticks);
            motorRight.setTargetPosition(motorRight.getCurrentPosition() - Ticks);
            motorLeft2.setTargetPosition(motorLeft2.getCurrentPosition() - Ticks);
            motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + Ticks);
        }
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(0.05);
        motorRight.setPower(0.05);
        motorRight2.setPower(0.05);
        motorLeft2.setPower(0.05);
        Thread.sleep(200);
        motorLeft.setPower(motorPower);
        motorRight.setPower(motorPower);
        motorRight2.setPower(motorPower);
        motorLeft2.setPower(motorPower);
    }

    void Turn90( boolean turnLeft, double motorPower)    {
        int Ticks = 915 ;
        if (turnLeft) {
            motorRight2.setTargetPosition(motorRight2.getCurrentPosition() + Ticks);
            motorRight.setTargetPosition(motorRight.getCurrentPosition() + Ticks);
            motorLeft2.setTargetPosition(motorLeft2.getCurrentPosition() - Ticks);
            motorLeft.setTargetPosition(motorLeft.getCurrentPosition() - Ticks);
        } else {
            motorRight2.setTargetPosition(motorRight2.getCurrentPosition() - Ticks);
            motorRight.setTargetPosition(motorRight.getCurrentPosition() - Ticks);
            motorLeft2.setTargetPosition(motorLeft2.getCurrentPosition() + Ticks);
            motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + Ticks);
        }
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(motorPower);
        motorRight.setPower(motorPower);
        motorRight2.setPower(motorPower);
        motorLeft2.setPower(motorPower);
    }

    //TPI is ticks per inches
    int ConvertInchesToTicks(double inches) {
        return (int) (inches * TPI);
    }
    int ConvertInchesToTicksForHang(double inches) { return (int) (inches * 306 / HangInchesPerRev); }

    void RunMotors(double inches, double motorPower) {
        int Ticks = ConvertInchesToTicks(inches);
        motorRight2.setTargetPosition(motorRight2.getCurrentPosition() + Ticks);
        motorRight.setTargetPosition(motorRight.getCurrentPosition() + Ticks);
        motorLeft2.setTargetPosition(motorLeft2.getCurrentPosition() + Ticks);
        motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + Ticks);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(motorPower);
        motorRight.setPower(motorPower);
        motorRight2.setPower(motorPower);
        motorLeft2.setPower(motorPower);
    }

    void RunMotorHang(double inches, double hangPower) {
        int Ticks = ConvertInchesToTicksForHang(inches);
        motorHang.setTargetPosition(motorHang.getCurrentPosition() + Ticks);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(hangPower);
    }
}
