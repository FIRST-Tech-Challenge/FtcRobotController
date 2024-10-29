package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot extends LinearOpMode {
    public IMU imu;
    public VisionPortal visionPortal;
    public Servo LA, RA, LH, RH, ALL, ARL, IT, DP, ADP;
    public DcMotorEx FL, FR, BL, BR, RL, LL,  encoder1, encoder2, encoder3 ;
    public int FL_Target, FR_Target, BL_Target, BR_Target;
    public final double[] tileSize            = {60.96, 60.96};  // Width * Length
    /* TETRIX Motor Encoder per revolution */
    public final int      Counts_per_TETRIX   = 24;
    /** HD HEX Motor Encoder per revolution */
    public final int      Counts_per_HD_HEX   = 28;
    /** 20:1 HD HEX Motor Encoder per revolution */
    public final int      Gear_20_HD_HEX      = Counts_per_HD_HEX * 20;
    /** (3 * 4 * 5):1 UltraPlanetary HD HEX Motor Encoder per revolution */
    public final double   Gear_60_HD_HEX      = Counts_per_HD_HEX * 54.8;
    public final double   Wheel_Diameter_Inch = 7.5/2.54;
    public final double   Counts_per_Inch     = Gear_20_HD_HEX / (Wheel_Diameter_Inch * Math.PI);
    public double[]       currentXY           = {0, 0};
    public final double   L                   = 32.9; //distance between 1 and 2 in cm
    public final double   B                   = 9.7; //distance between center of 1 and 2 and 3 in cm
    public final double   r                   = 2.4 ; // Odomentry wheel radius in cm
    public final double   N                   = 2000.0 ; // ticks per one rotation
    public double         cm_per_tick     = 2.0 * Math.PI * r / N ;
    public double         theta, Posx, Posy, heading, n, CurPosLift, Lift_Power, dn1, dn2, dn3, dyaw ;
    //    // update encoder
    int                   left_encoder_pos , right_encoder_pos , center_encoder_pos ,
                          prev_left_encoder_pos, prev_right_encoder_pos, prev_center_encoder_pos = 0;
    double                CurrentYaw, OldYaw         = 0;


    public void Odomentry() {
        left_encoder_pos = -encoder1.getCurrentPosition();
        right_encoder_pos = -encoder2.getCurrentPosition();
        center_encoder_pos = encoder3.getCurrentPosition();

        double delta_left_encoder_pos = (left_encoder_pos - prev_left_encoder_pos) * cm_per_tick;
        double delta_right_encoder_pos = (right_encoder_pos - prev_right_encoder_pos) * cm_per_tick;
        double delta_center_encoder_pos = (center_encoder_pos - prev_center_encoder_pos) * cm_per_tick;

        double phi = (delta_right_encoder_pos - delta_left_encoder_pos) / L;
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2.0;
        double delta_perp_pos = delta_center_encoder_pos - B * phi;

        double delta_x = delta_perp_pos * Math.cos(heading) - delta_middle_pos * Math.sin(heading);
        double delta_y = delta_perp_pos * Math.sin(heading) + delta_middle_pos * Math.cos(heading);

        Posx += delta_x;
        Posy += delta_y;
        heading += phi;

        heading = WrapRads(heading);

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;

//        CurrentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        double alpha = 0.8;
//        dn1 = alpha * dn1 + (1 - alpha) * (Old1Position - Current1Position) * cm_per_tick;
//        dn2 = alpha * dn2 + (1 - alpha) * (Old2Position - Current2Position) * cm_per_tick;
//        dn3 = (Current3Position - Old3Position) * cm_per_tick;
//        dyaw = CurrentYaw - OldYaw;



//        double dy = (dn1 + dn2) / 2;
//        double dx = dn3 - B * dyaw;
//
//        double deltaY = dy * Math.cos(CurrentYaw) - dx * Math.sin(CurrentYaw);
//        double deltaX = dy * Math.sin(CurrentYaw) + dx * Math.cos(CurrentYaw);
//
//        Posy -= deltaY;
//        Posx += deltaX;
//        theta += (dn1 - dn2) / L;
//
//        Old1Position = Current1Position;
//        Old2Position = Current2Position;
//        Old3Position = Current3Position;
//        OldYaw = CurrentYaw;
    }

    public void move(double tilex, double tiley, double setpoint, double[] basespeed, double[] Kpidf_X, double[] Kpidf_Y) {
        Controller  pidR    = new Controller(2.4, 0.8, 0.0, 0, 0.1, toRadian(0.75));
        Controller  DelthaX = new Controller(Kpidf_X[0], Kpidf_X[1], Kpidf_X[2], Kpidf_X[3], basespeed[0], 0.5);
        Controller  DelthaY = new Controller(Kpidf_Y[0], Kpidf_Y[1], Kpidf_Y[2], Kpidf_Y[3], basespeed[1], 0.5);
        double targetx = tilex * tileSize[0];
        double targety = tiley * tileSize[1];
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        int IS_Complete = 0;
        while (opModeIsActive()) {
            Odomentry();
//            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double Vx = DelthaX.Calculate((targetx - Posx)*-1);
            double Vy = DelthaY.Calculate(targety - Posy);

            double x2    =  (Math.cos(heading) * Vx) - (Math.sin(heading) * Vy);
            double y2    =  (Math.sin(heading) * Vx) + (Math.cos(heading) * Vy);

            double r =  pidR.Calculate(WrapRads(toRadian(setpoint) - heading));
            double d = Math.max(Math.abs(Vx) + Math.abs(Vy), 1);
            MovePower((y2 - x2 - r) / d, (y2 + x2 + r) / d,
                    (y2 + x2 - r) / d, (y2 - x2 + r) / d);
//            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("XY", "%6f cm %6f cm" , Posx, Posy);
            telemetry.addData("tagetXtargetY", "%6f cm %6f cm" , targetx, targety);
            telemetry.addData("X", "%6f cm/s %6f cm" , Vx, DelthaX.ErrorTolerance);
            telemetry.addData("Y", "%6f cm/s %6f cm" , Vy, DelthaY.ErrorTolerance);
            telemetry.addData("ErrorX", DelthaX.Error);
            telemetry.addData("ErrorY", DelthaY.Error);
            telemetry.addData("Complete", IS_Complete);
            telemetry.update();
            if (Vx == 0.0 && Vy == 0 && r == 0) {
                IS_Complete += 1;
                if (IS_Complete > 100) break;
                continue;
            }
            IS_Complete = 0;
        }
        Break(0.5);
    }


    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left * 0.9);
        FR.setPower(Front_Right * 0.9);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }
    public void Break(double stopSecond) {
        if (stopSecond == 0) return;
        MovePower(0, 0, 0, 0);
        MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep((long) (stopSecond * 1000));
    }

    public void MoveMode(DcMotor.RunMode moveMode) {
        FL.setMode(moveMode);
        FR.setMode(moveMode);
        BL.setMode(moveMode);
        BR.setMode(moveMode);
    }
    public void Initialize(DcMotor.RunMode moveMode, double[] DuoServoAng, double[] ServoAng) {
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");    FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");     BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        encoder1 = FL ;
        encoder2 = FR;
        encoder3 = BL;

        // Initialize IMU
      imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection .RIGHT)));
        // Reverse Servo
        // Set Servo Position
        // setMode Motors
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower Motors
        MovePower(0, 0, 0, 0);
    }

}