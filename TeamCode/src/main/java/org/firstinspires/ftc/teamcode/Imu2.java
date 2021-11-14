package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.selfDrive.AutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.selfDrive.AutoDriveUtils.logLine;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous(name="Turning", group="Pushbot")
public class Imu2 extends BaseOpMode {
    public Hardware2 robot = new Hardware2(true);  // We are using encoders, so pass it ????
    private ElapsedTime runtime = new ElapsedTime();
    //instantiating variable imu of type BNO055IMU
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        //call imu.initialize() and feed it parameters
        while(!opModeIsActive()){
            imu.initialize(parameters);
        }
        waitForStart();
        //write all run code here
        if (opModeIsActive()) {
            turnRight(90, 0.1);
        }
    }
    public void turnRight(double degrees, double power){
        degrees *= -1;
        //Angle before you turn
        double startingAngle = getHeading();
        //Setting motors to turn right
        robot.getLeftDrive().setPower(power);
        robot.getRightDrive().setPower(-power);
        //Waiting until the change in degrees is greater than desired change in degrees

        while ((getHeading()-startingAngle)>=degrees && opModeIsActive()){
            logData(this, "Still In While loop", String.format("Heading at: %.2f, Started at: %.2f", getHeading(), startingAngle));
        }
        //stop all motors
        robot.getLeftDrive().setPower(0);
        robot.getRightDrive().setPower(0);
    }
    public void turnLeft(double degrees, double power){
        //Angle before you turn
        double startingAngle = getHeading();
        //Setting motors to turn right
        robot.getLeftDrive().setPower(-power);
        robot.getRightDrive().setPower(power);
        //Waiting until the change in degrees is greater than desired change in degrees
        while ((getHeading()-startingAngle)<=degrees && opModeIsActive()){
            logData(this, "Still In While loop", String.format("Heading at: %.2f, Started at: %.2f", getHeading(), startingAngle));
        }
        //stop all motors
        robot.getLeftDrive().setPower(0);
        robot.getRightDrive().setPower(0);
    }
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }

    @Override
    public Hardware2 getRobot() {
        return robot;
    }
}