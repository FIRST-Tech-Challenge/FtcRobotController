package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoMethods {
    double TicsPerRevolution = 537.6;
    double Circumference = 11.87;
    double HangInchesPerRev = 0.25;
    double TPI = TicsPerRevolution / Circumference;
    double StrafeTPI = 50.2512563;
    double TicksPerDeg = 10.17;
    double DegAtBoard = -25;
    private DcMotor motorLeft, motorLeft2, motorRight, motorRight2, motorIntake, motorHang;
    private int leftTarget,left2Target,rightTarget,right2Target;

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
            right2Target = motorRight2.getCurrentPosition() - Ticks;
            rightTarget = motorRight.getCurrentPosition() + Ticks;
            left2Target = motorLeft2.getCurrentPosition() + Ticks;
            leftTarget = motorLeft.getCurrentPosition() - Ticks;
        } else {
            right2Target = motorRight2.getCurrentPosition() + Ticks;
            rightTarget = motorRight.getCurrentPosition() - Ticks;
            left2Target = motorLeft2.getCurrentPosition() - Ticks;
            leftTarget = motorLeft.getCurrentPosition() + Ticks;
        }
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(motorPower);
        }
        ZeroMotors();
    }

    void Strafe( boolean sendright, double motorPower){
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (sendright) {
            motorLeft.setPower(-motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(-motorPower);
            motorLeft2.setPower(motorPower);
        } else {
            motorLeft.setPower(motorPower);
            motorRight.setPower(-motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(-motorPower);
        }

    }

    void Turn90( boolean turnLeft, double motorPower)    {
        int Ticks = 915 ;
        if (turnLeft) {
            right2Target = motorRight2.getCurrentPosition() + Ticks;
            rightTarget = motorRight.getCurrentPosition() + Ticks;
            left2Target = motorLeft2.getCurrentPosition() - Ticks;
            leftTarget = motorLeft.getCurrentPosition() - Ticks;
        } else {
            right2Target = motorRight2.getCurrentPosition() - Ticks;
            rightTarget = motorRight.getCurrentPosition() - Ticks;
            left2Target = motorLeft2.getCurrentPosition() + Ticks;
            leftTarget = motorLeft.getCurrentPosition() + Ticks;
        }
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(motorPower);
        }
        ZeroMotors();
    }

    //TPI is ticks per inches
    int ConvertInchesToTicks(double inches) {
        return (int) (inches * TPI);
    }
    int ConvertInchesToTicksForHang(double inches) { return (int) (inches * 306 / HangInchesPerRev); }

    void RunMotors(double inches, double motorPower) {
        int Ticks = ConvertInchesToTicks(inches);
        leftTarget = motorLeft.getCurrentPosition() + Ticks;
        left2Target = motorLeft2.getCurrentPosition() + Ticks;
        rightTarget = motorRight.getCurrentPosition() + Ticks;
        right2Target = motorRight2.getCurrentPosition() + Ticks;
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(motorPower);
        }
        ZeroMotors();
    }

    void RunMotorHang(double inches, double hangPower) {
        int Ticks = ConvertInchesToTicksForHang(inches);
        motorHang.setTargetPosition(motorHang.getCurrentPosition() + Ticks);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(hangPower);
    }
    void ZeroMotors(){
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight2.setPower(0);
        motorLeft2.setPower(0);
    }
    private boolean AtTarget() {
        return motorLeft.getCurrentPosition() > leftTarget - 10 && motorLeft.getCurrentPosition() < leftTarget + 10 &&
                motorLeft2.getCurrentPosition() > left2Target - 10 && motorLeft2.getCurrentPosition() < left2Target + 10 &&
                motorRight.getCurrentPosition() > rightTarget - 10 && motorRight.getCurrentPosition() < rightTarget + 10 &&
                motorRight2.getCurrentPosition() > right2Target - 10 && motorRight2.getCurrentPosition() < right2Target + 10;
    }

    void SquareOnTag (double x, double pitch, double motorPower){
        int rightTicks, right2Ticks, leftTicks, left2Ticks;
        int strafeTick = Math.abs(StrafeInchesToTicks(x-2));
        double degree = pitch - DegAtBoard;
        int angleTicks = Math.abs((int)( degree * TicksPerDeg));
        boolean strafeRight = x-2 >= 0;
        boolean turnRight = degree >= 0;
        if(strafeRight && turnRight){
            right2Ticks =  -strafeTick - angleTicks;
            rightTicks =  strafeTick - angleTicks;
            left2Ticks = strafeTick + angleTicks;
            leftTicks = -strafeTick + angleTicks;
        }
        else if(strafeRight){
            right2Ticks = -strafeTick + angleTicks;
            rightTicks = strafeTick + angleTicks;
            left2Ticks = strafeTick - angleTicks;
            leftTicks = -strafeTick - angleTicks;
        }
        else if(turnRight){
            right2Ticks = strafeTick - angleTicks;
            rightTicks = -strafeTick - angleTicks;
            left2Ticks = -strafeTick + angleTicks;
            leftTicks = strafeTick + angleTicks;
        }
        else{
            right2Ticks = strafeTick + angleTicks;
            rightTicks = -strafeTick + angleTicks;
            left2Ticks = -strafeTick - angleTicks;
            leftTicks = strafeTick - angleTicks;
        }
        int maxTicks = Math.max(left2Ticks, Math.max(leftTicks, Math.max(Math.abs(right2Ticks), Math.abs(rightTicks))));
        motorRight2.setTargetPosition(motorRight2.getTargetPosition() + right2Ticks);
        motorRight.setTargetPosition(motorRight.getCurrentPosition() + rightTicks);
        motorLeft2.setTargetPosition(motorLeft2.getCurrentPosition() + left2Ticks);
        motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + leftTicks);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower * Math.abs(leftTicks) / maxTicks);
            motorRight.setPower(motorPower * Math.abs(rightTicks) / maxTicks);
            motorRight2.setPower(motorPower * Math.abs(right2Ticks) / maxTicks);
            motorLeft2.setPower(motorPower * Math.abs(left2Ticks) / maxTicks);
        }
        ZeroMotors();
    }

}
