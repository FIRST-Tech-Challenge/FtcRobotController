package org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base;


import org.firstinspires.ftc.teamcode.AutoCode.Control.AcceleratedMotorPowerGenerator;
import org.firstinspires.ftc.teamcode.drivebase.MotorPowers;

public class AccerlationControlledDrivetrainPowerGeneratorForAuto
{
    private AcceleratedMotorPowerGenerator accelFl, accelFr, accelRl, accelRr;
    private MotorPowers reusablePowsObj = new MotorPowers();

    public AccerlationControlledDrivetrainPowerGeneratorForAuto(double accelerationRate, double decelerationRate, double minPow)
    {
        accelFl = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
        accelFr = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
        accelRl = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
        accelRr = new AcceleratedMotorPowerGenerator(accelerationRate, decelerationRate, minPow);
    }

    public MotorPowers getAccelerationControlledPowers(MotorPowers input)
    {
        accelFl.setTargetPower(input.frontLeft);
        accelFr.setTargetPower(input.frontRight);
        accelRl.setTargetPower(input.rearLeft);
        accelRr.setTargetPower(input.rearRight);

        accelFl.update();
        accelFr.update();
        accelRl.update();
        accelRr.update();

        reusablePowsObj.frontLeft = accelFl.getAccelerationControlledPower();
        reusablePowsObj.frontRight = accelFr.getAccelerationControlledPower();
        reusablePowsObj.rearLeft = accelRl.getAccelerationControlledPower();
        reusablePowsObj.rearRight = accelRr.getAccelerationControlledPower();

        return reusablePowsObj;
    }

    public void clr()
    {
        accelFl.stopNow();
        accelFr.stopNow();
        accelRl.stopNow();
        accelRr.stopNow();
    }
}
