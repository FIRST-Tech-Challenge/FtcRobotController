package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PlotVoltageToSpeed", group = "Teleops")
public class PlotVoltageToSpeed extends OpMode {

    // FTC Dashboard stuff
    Telemetry m_dashTelemetry;


    public static double POWER_APPLIED = .4;

    // Motors for drivetrain
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    ElapsedTime time = new ElapsedTime();

    double prevTime = 0;

    double prevFRAngle = 0;
    double prevBRAngle = 0;
    double prevBLAngle = 0;
    double prevFLAngle = 0;

    double totalAngSpeed = 0;
    double courseLength = 2;
    double startTime = 0.75;

    // Dash Constants
    public static double PPR = 2000;


    public void init()
    {
        // Drivetrain initialization
        frontLeftMotor = hardwareMap.dcMotor.get("frontL");
        backLeftMotor = hardwareMap.dcMotor.get("backL");
        frontRightMotor = hardwareMap.dcMotor.get("frontR");
        backRightMotor = hardwareMap.dcMotor.get("backR");

        m_dashTelemetry = createDashTelem();
    }

    public void addAngularSpeed(double angSpeed, double dt)
    {
        totalAngSpeed += angSpeed * dt;
    }

    public Telemetry createDashTelem()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        return dashboard.getTelemetry();
    }

    public void loop()
    {
        if (time.seconds() > startTime)
        {
            if (time.seconds() < startTime + courseLength) {
                double deltaTime = time.seconds() - prevTime;
                double deltaFRAngle = (frontRightMotor.getCurrentPosition() - prevFRAngle) / PPR;
                double deltaFLAngle = (frontLeftMotor.getCurrentPosition() - prevFLAngle) / PPR;
                double deltaBRAngle = (backRightMotor.getCurrentPosition() - prevBRAngle) / PPR;
                double deltaBLAngle = (backLeftMotor.getCurrentPosition() - prevBLAngle) / PPR;

                double FRAngularSpeed = Math.abs(deltaFRAngle / deltaTime);
                double FLAngularSpeed = Math.abs(deltaFLAngle / deltaTime);
                double BRAngularSpeed = Math.abs(deltaBRAngle / deltaTime);
                double BLAngularSpeed = Math.abs(deltaBLAngle / deltaTime);

                m_dashTelemetry.addData("angSpeed", BLAngularSpeed);
                m_dashTelemetry.update();

                double totalAngularSpeed = (FRAngularSpeed + FLAngularSpeed + BRAngularSpeed + BLAngularSpeed) / 1.0;
                addAngularSpeed(totalAngularSpeed, deltaTime);
            }
            else
            {
                double finalSpeed = totalAngSpeed / courseLength;
                m_dashTelemetry.addData("angSpeed", finalSpeed);
                m_dashTelemetry.update();
            }
        }
        prevBRAngle = backRightMotor.getCurrentPosition();
        prevFRAngle = frontRightMotor.getCurrentPosition();
        prevBLAngle = backLeftMotor.getCurrentPosition();
        prevFLAngle = frontLeftMotor.getCurrentPosition();
    }

    public void stop() {

    }


}
