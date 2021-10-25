package org.firstinspires.ftc.teamcode.utils;

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/DrivetrainManager4WD.java
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
========
public abstract class Drivetrain {
>>>>>>>> testing-Michael-Lachut:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/Drivetrain.java

    public abstract void driveDistance(int rightDistance, int leftDistance, int speed);

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/DrivetrainManager4WD.java
public class DrivetrainManager4WD {
========
    public abstract void driveWithEncoder(int rightSpeed, int leftSpeed);
>>>>>>>> testing-Michael-Lachut:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/Drivetrain.java

    public abstract void driveWithoutEncoder(int rightPower, int leftPower);

    public abstract void driveDistance(int distance, int speed);

    public abstract void driveWithEncoder(int speed);

    public abstract void driveWithoutEncoder(int power);

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/DrivetrainManager4WD.java
    public DrivetrainManager4WD(List<String> driveMotors, HardwareMap hardwareMap) {
        ld1_name = driveMotors.get(0);
        rd1_name = driveMotors.get(1);
        ld2_name = driveMotors.get(2);
        rd2_name = driveMotors.get(3);
========
    public abstract void stop();
>>>>>>>> testing-Michael-Lachut:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/Drivetrain.java

    public abstract void brake();

    public abstract void reset();

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/DrivetrainManager4WD.java
    public void SetPower(double ld1, double rd1, double ld2, double rd2) {
        this.ld1.setPower(ld1);
        this.ld2.setPower(ld2);
        this.rd1.setPower(rd1);
        this.rd2.setPower(rd2);
    }

    public void SetPower(double left, double right) {
        SetPower(left, right, left, right);
    }

    public void Translate(double powerY) {
        SetPower(powerY, powerY, powerY, powerY);
    }

    /**
    * @param direction - Set to -1 to turn counterclockwise, otherwise set as 1
     */
    public void Rotate(double power, int direction) {
        SetPower(direction*power, -direction*power);
    }

    public void EvalGamepad(double x, double y) {
        double drive = -y;
        double turn  =  x;
        double left    = Range.clip(drive + turn, -100.0, 100.0) ;
        double right   = Range.clip(drive - turn, -100.0, 100.0) ;
        SetPower(left, right);
    }
========
>>>>>>>> testing-Michael-Lachut:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/Drivetrain.java
}
