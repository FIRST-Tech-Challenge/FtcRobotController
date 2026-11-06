
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleOp_2026_2027", group="Iterative OpMode")
public class TeleOp_2026_2027 extends OpMode {


    //---Declare Motors---//
    //private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    private DcMotor intakeMotor;
    private DcMotor launcherMotor;

    IMU imu;

    private double yaw;

    boolean intakeOn = false;
    boolean launcherOn = false;


    //---Code to run ONCE when the driver hits INIT---//
    @Override
    public void init()
    {
        //---Initialize Motors---//
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        //---Initialize
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        //---Set Motor Directions---//
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        //---Add telemetry data---//
        telemetry.addData("Status", "Initialized");
    }



    //---Code to run REPEATEDLY after the driver hits INIT, but before they hit START---//
    @Override

    public void init_loop()
    {



    }

    //---Code to run ONCE when the driver hits START---//
    @Override
    public void start()
    {
        //runtime.reset();
    }

    //---Code to run REPEATEDLY after the driver hits START but before they hit STOP---//
    @Override
    public void loop()
    {
        displayIMU();

        driveBase();

        launcher();
        intake();
    }

    //---Code to run ONCE after the driver hits STOP---//
    @Override
    public void stop()
    {

    }


    public void launcher()
    {
        launcherOn = gamepad1.dpad_up;

        if(launcherOn){
            launcherMotor.setPower(0);
        }
        else{
            launcherMotor.setPower(0);
        }
    }

    public void intake()
    {
        if(gamepad1.aWasPressed()){
            intakeOn = !intakeOn;
        }

        if(intakeOn){
            intakeMotor.setPower(1);
        }
        else{
            launcherMotor.setPower(0);
        }

    }

    public void hood()
    {

    }

    public void driveBase()
    {
        //Get variables from the gamepad
        double y_Input = gamepad1.left_stick_y;
        double x_Input = gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;

        double x_Mod = x_Input * Math.cos(yaw) + y_Input * Math.sin(yaw);
        double y_Mod = -x_Input * Math.sin(yaw) + y_Input * Math.cos(yaw);

        //Translate inputs to outputs for motors
        motorBL.setPower ((y_Mod + driveTurn) - x_Mod);
        motorBR.setPower ((y_Mod - driveTurn) + x_Mod);
        motorFL.setPower ((y_Mod + driveTurn) + x_Mod);
        motorFR.setPower ((y_Mod - driveTurn) - x_Mod);

    }

    public void displayIMU()
    {

        //telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            yaw = orientation.getYaw(AngleUnit.RADIANS);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
    }

    /*public void routineBlock()
    {
        String routineString = routine.toString();
        routineString = routineString.substring(1, routineString.length()-1);

        File file = AppUtil.getInstance().getSettingsFile("Saving_and_Loading_on_text_file");
        ReadWriteFile.writeFile(file, routineString);

        requestOpModeStop();
    }*/

}
