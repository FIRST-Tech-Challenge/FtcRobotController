package org.firstinspires.ftc.teamcode.AutoCode.Control;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivebase.DriveTrain;
import org.firstinspires.ftc.teamcode.drivebase.MotorPowers;

public class AccelerationControlledDrivetrain implements DriveTrain
{
    private DriveTrain driveTrain;
    private AcceleratedMotorPowerGenerator accelFl, accelFr, accelRl, accelRr;

    public AccelerationControlledDrivetrain(DriveTrain driveTrain, double accelerationRate, double decelerationRate, double minPow)
    {
        this.driveTrain = driveTrain;

        accelFl = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
        accelFr = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
        accelRl = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
        accelRr = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
    }

    @Override
    public void init(HardwareMap hardwareMap)
    {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMotorPowers(MotorPowers pows)
    {
        setMotorPowers(pows.frontLeft, pows.frontRight, pows.rearLeft, pows.rearRight);
    }

    @Override
    public void setMotorPowers(double pow)
    {
        setMotorPowers(pow, pow, pow, pow);
    }

    @Override
    public void setMotorPowers(double left, double right)
    {
        setMotorPowers(left, right, left, right);
    }

    public void stopNow()
    {
        accelFl.stopNow();
        accelFr.stopNow();
        accelRl.stopNow();
        accelRr.stopNow();

        driveTrain.setMotorPowers(0);
    }

    public void setMotorPowers(double fl, double fr, double rl, double rr)
    {
        accelFl.setTargetPower(fl);
        accelFr.setTargetPower(fr);
        accelRl.setTargetPower(rl);
        accelRr.setTargetPower(rr);

        accelFl.update();
        accelFr.update();
        accelRl.update();
        accelRr.update();

        double setFl = accelFl.getAccelerationControlledPower();
        double setFr = accelFr.getAccelerationControlledPower();
        double setRl = accelRl.getAccelerationControlledPower();
        double setRr = accelRr.getAccelerationControlledPower();

        //System.out.println(String.format("AcclCtrlDT: setting powers; FL=%f, FR=%f, RL=%f, RR=%f", setFl, setFr, setRl, setRr));

        driveTrain.setMotorPowers(setFl, setFr, setRl, setRr);
    }

    @Override
    public void enableBrake(boolean brake)
    {
        driveTrain.enableBrake(brake);
    }

    @Override
    public void stopMotors()
    {
        driveTrain.stopMotors();
    }

    @Override
    public void enablePID()
    {
        driveTrain.enablePID();
    }

    @Override
    public void disablePID()
    {
        driveTrain.disablePID();
    }

    @Override
    public void resetEncoders()
    {
        driveTrain.resetEncoders();
    }

    @Override
    public void softResetEncoders()
    {
        driveTrain.softResetEncoders();
    }

    @Override
    public int encoderFrontLeft()
    {
        return driveTrain.encoderFrontLeft();
    }

    @Override
    public int encoderFrontRight()
    {
        return driveTrain.encoderFrontRight();
    }

    @Override
    public int encoderRearLeft()
    {
        return driveTrain.encoderRearLeft();
    }

    @Override
    public int encoderRearRight()
    {
        return driveTrain.encoderRearRight();
    }
}
