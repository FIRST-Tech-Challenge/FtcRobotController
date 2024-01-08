package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Blue Front", group="Linear Opmode")
public class BlueFront extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    private ColorSensor color = null;

    private IMU imu = null;

    private int spikeMark = 0;

    private int[][] destinations;
    private int index = 0;

    int posX = 0;
    int posY = 1;

    int destX = 0;
    int destY = 0;

    YawPitchRollAngles ypr;

    double rotation;

    double x1 = 0;
    double y1 = 0;
    double x2 = 0;

    @Override
    public void runOpMode() {
        leftDriveFront  = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");

        color = hardwareMap.get(ColorSensor.class, "color");

        imu = hardwareMap.get(IMU.class, "imu");

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        ypr = imu.getRobotYawPitchRollAngles();
        rotation = ypr.getYaw(AngleUnit.DEGREES);

        destinations = new int[][]{{2, 1}, {2, 5}};

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            x1 = 0;
            y1 = 0;
            x2 = 0;

            if(index < destinations.length){
                destX = destinations[index][0];
                destY = destinations[index][1];
            }

            if(posX != destX || posY != destY){
                if(runtime.time() < (Math.abs(destX - posX) + Math.abs(destY - posY)) * 0.9){
                    if(rotation < 0.1 && rotation > -0.1){
                        if(destX < posX){
                            y1 = 1;
                        }
                        if(destX > posX){
                            y1 = -1;
                        }
                        if(destY < posY){
                            x1 = 1;
                        }
                        if(destY > posY){
                            x1 = -1;
                        }
                    }
                    if(rotation < 90.1 && rotation > 89.9){
                        if(destX < posX){
                            x1 = 1;
                        }
                        if(destX > posX){
                            x1 = -1;
                        }
                        if(destY < posY){
                            y1 = -1;
                        }
                        if(destY > posY){
                            y1 = 1;
                        }
                    }
                    if(rotation < 180.1 && rotation > 179.9){
                        if(destX < posX){
                            y1 = -1;
                        }
                        if(destX > posX){
                            y1 = 1;
                        }
                        if(destY < posY){
                            x1 = -1;
                        }
                        if(destY > posY){
                            x1 = 1;
                        }
                    }
                    if(rotation < 270.1 && rotation > 269.9){
                        if(destX < posX){
                            x1 = -1;
                        }
                        if(destX > posX){
                            x1 = 1;
                        }
                        if(destY < posY){
                            y1 = 1;
                        }
                        if(destY > posY){
                            y1 = -1;
                        }
                    }
                }else{
                    x1 = 0;
                    y1 = 0;
                    x2 = 0;

                    posX = destX;
                    posY = destY;

                    index += 1;

                    runtime.reset();
                }

                /*if(ypr.getYaw(AngleUnit.DEGREES) < rotation - 0.1){
                    x2 = 0.01;
                }
                if(ypr.getYaw(AngleUnit.DEGREES) > rotation + 0.1){
                    x2 = -0.01;
                }*/
            }
            //drive();

            telemetry.addData("X1", x1);
            telemetry.addData("Y1", y1);
            telemetry.addData("X2", x2);
            telemetry.addData("DestX", destX);
            telemetry.addData("DestY", destY);
            telemetry.addData("PosX", posX);
            telemetry.addData("PosY", posY);
            telemetry.addData("Rotation", ypr.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Color", String.valueOf(color.red()) + ", " + String.valueOf(color.green()) + ", " + String.valueOf(color.blue()));
            telemetry.update();
        }
    }

    void drive(){
        double fl = 0.0;
        double fr = 0.0;
        double bl = 0.0;
        double br = 0.0;

        fl += y1;
        fr += y1;
        bl += y1;
        br += y1;

        fl -= x1;
        fr += x1;
        bl += x1;
        br -= x1;

        fl -= x2;
        fr += x2;
        bl -= x2;
        br += x2;

        leftDriveFront.setPower(fl / 2);
        rightDriveFront.setPower(fr / 2);
        leftDriveBack.setPower(bl / 2);
        rightDriveBack.setPower(br / 2);
    }
}
