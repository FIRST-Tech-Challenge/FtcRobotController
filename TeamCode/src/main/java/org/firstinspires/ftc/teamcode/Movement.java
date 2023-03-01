package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Movement {
    public void reset(HardwarePushbot robot){
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void move(HardwarePushbot robot){
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Forward(int distance, HardwarePushbot robot, double power){
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;
        frontLeftPosition += distance;
        frontRightPosition += distance;
        backLeftPosition -= distance;
        backRightPosition -= distance;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);


    }

    public void Backward(int distance, HardwarePushbot robot, double power){
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;
        frontLeftPosition -= distance;
        frontRightPosition -= distance;
        backLeftPosition += distance;
        backRightPosition += distance;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);



    }
    public void Right(int distance, HardwarePushbot robot, double power){
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;
        frontLeftPosition += distance;
        frontRightPosition -= distance;
        backLeftPosition += distance;
        backRightPosition -= distance;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);

    }
    public void Left(int distance, HardwarePushbot robot, double power){
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;
        frontLeftPosition -= distance;
        frontRightPosition += distance;
        backLeftPosition -= distance;
        backRightPosition += distance;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);

    }


    public void Rrotate (int distance, HardwarePushbot robot, double power){
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;
        frontLeftPosition += distance*10;
        frontRightPosition -= distance*10;
        backLeftPosition -= distance*10;
        backRightPosition += distance*10;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);

    }
    public void Lrotate (int distance, HardwarePushbot robot, double power){
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;
        frontLeftPosition -= distance*10;
        frontRightPosition += distance*10;
        backLeftPosition += distance*10;
        backRightPosition -= distance*10;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);

    }
    public void ScapeGoat(int distance,HardwarePushbot robot, double power){
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;
        frontLeftPosition += distance;
        frontRightPosition -= distance;
        backLeftPosition += distance;
        backRightPosition -= distance;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);

    }
}
