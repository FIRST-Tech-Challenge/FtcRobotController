package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Turning", group="Pushbot")
public class Imu2 extends LinearOpMode {

    public DMHardware robot = new DMHardware(true);  // We are using encoders, so pass it ????
    private ElapsedTime runtime = new ElapsedTime();
    //instantiating variable imu of type BNO055IMU
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleOpIMU(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //call imu.initialize() and feed it parameters
        imu.initialize(parameters);
        waitForStart();
        //write all run code here
        turnRight(90, 0.1);
        sleep(1000);
        //turnLeft(-90, 0.4);

    }
    public void turnRight(double degrees, double power){
        //Angle before you turn
        double startingAngle = getHeading();
        telemetry.addLine("Hello");
        telemetry.addData("startingAngle", getHeading());
        telemetry.update();
        sleep(3000);
        //Setting motors to turn right
        robot.leftMotor.setPower(-power);
        robot.rightMotor.setPower(power);
        //Waiting until the change in degrees is greater than desired change in degrees
        while ((Math.abs(getHeading())-Math.abs(startingAngle)) <= degrees && opModeIsActive()){
            telemetry.addData("Heading", getHeading());
            telemetry.update();
            //sleep(3000);
        }

    }
    public void turnLeft(double degrees, double power){
        //Angle before you turn
        double startingAngle = getHeading();
        //Setting motors to turn right
        robot.leftMotor.setPower(-power);
        robot.rightMotor.setPower(power);
        //Waiting until the change in degrees is greater than desired change in degrees
        while ((getHeading()-startingAngle )<= degrees && opModeIsActive()){
        }
        //stop all motors
    }
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }
}