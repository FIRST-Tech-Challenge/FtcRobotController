package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {
    Telemetry telemetry;
    public DcMotor FrontLeft;
    public DcMotor BackLeft;
    public DcMotor FrontRight;
    public DcMotor BackRight;

    double PULSES_PER_ROTATION = 537.5;
    double GEAR_RATIO = 1;

    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        setup(hardwareMap);
    }
    public void setup(HardwareMap hardwareMap) {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        setMotorDirection(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE );
    }

    public int convertDistTicks( double distanceToTravel, double circumference ) {
        double revolutions = distanceToTravel / circumference;
        int totalTicks = (int) Math.round( (revolutions * PULSES_PER_ROTATION) / GEAR_RATIO );

        return totalTicks;
    }

    public int convertTicksDist( double ticksToTravel, double circumference ) {
        double calculations = ticksToTravel * circumference * GEAR_RATIO;
        int totalDistance = (int) Math.round( calculations / PULSES_PER_ROTATION );

        return totalDistance;
    }

    public void setMotorDirection(DcMotorSimple.Direction flDirection, DcMotorSimple.Direction frDirection, DcMotorSimple.Direction blDirection, DcMotorSimple.Direction brDirection){
        FrontLeft.setDirection(flDirection);
        BackLeft.setDirection(blDirection);
        FrontRight.setDirection(frDirection);
        BackRight.setDirection(brDirection);
    }

    public void setMotorPower( double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower ) {
        FrontLeft.setPower( frontLeftPower );
        BackLeft.setPower( backLeftPower );
        FrontRight.setPower( frontRightPower );
        BackRight.setPower( backRightPower );
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior flBehavior, DcMotor.ZeroPowerBehavior blBehavior, DcMotor.ZeroPowerBehavior frBehavior, DcMotor.ZeroPowerBehavior brBehavior){
        FrontLeft.setZeroPowerBehavior( flBehavior );
        BackLeft.setZeroPowerBehavior( blBehavior );
        FrontRight.setZeroPowerBehavior( frBehavior );
        BackRight.setZeroPowerBehavior( brBehavior );
    }
    public void setRunMode(DcMotor.RunMode flMode, DcMotor.RunMode frMode, DcMotor.RunMode blMode, DcMotor.RunMode brMode ) {
        FrontLeft.setMode( flMode );
        BackLeft.setMode( blMode );
        FrontRight.setMode( frMode );
        BackRight.setMode( brMode );
    }
}
