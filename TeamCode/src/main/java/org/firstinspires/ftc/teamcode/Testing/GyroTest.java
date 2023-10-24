package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotClass;

@TeleOp
public class GyroTest extends LinearOpMode {
//    flags for testing
    public boolean runTurnCode = false;

    RobotClass robot = new RobotClass(this);
    public Orientation angles = null;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //hardware mapping and initializing imu
        while(!robot.imu.isGyroCalibrated()){
            sleep(10);
        }

        waitForStart();

        while (opModeIsActive()) {
            angles = robot.imu.getAngularOrientation();
            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();

            if (runTurnCode) {
                //turn 90 degrees using gyro
                double targetAngle = -90;
                if (angles.firstAngle >= targetAngle-0.5 && angles.firstAngle <= targetAngle+0.5){
                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                    sleep(1000);
                    if (angles.firstAngle >= targetAngle-0.5 && angles.firstAngle <= targetAngle+0.5) {
                        break;
                    }
                }else if (angles.firstAngle >= targetAngle+0.5){
                    if (angles.firstAngle <= targetAngle+10){
                        robot.frontLeft.setPower(-0.15);
                        robot.frontRight.setPower(0.15);
                        robot.backLeft.setPower(-0.15);
                        robot.backRight.setPower(0.15);
                    }else {
                        robot.frontLeft.setPower(-0.5);
                        robot.frontRight.setPower(0.5);
                        robot.backLeft.setPower(-0.5);
                        robot.backRight.setPower(0.5);
                    }
                }else if (angles.firstAngle <= targetAngle-0.5){
                    if (angles.firstAngle >= targetAngle-10){
                        robot.frontLeft.setPower(0.15);
                        robot.frontRight.setPower(-0.15);
                        robot.backLeft.setPower(0.15);
                        robot.backRight.setPower(-0.15);

                    }else{
                        robot.frontLeft.setPower(0.5);
                        robot.frontRight.setPower(-0.5);
                        robot.backLeft.setPower(0.5);
                        robot.backRight.setPower(-0.5);
                    }
                }
            }
        }
    }
}
