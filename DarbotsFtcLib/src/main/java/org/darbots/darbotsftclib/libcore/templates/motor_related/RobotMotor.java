package org.darbots.darbotsftclib.libcore.templates.motor_related;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public interface RobotMotor<E extends MotorType> extends RobotNonBlockingDevice {
    public static final double N_OVER_M_PER_KG_OVER_CM = 0.0980665;
    enum MovingType{
        toCount,
        withSpeed,
        reset
    }
    E getMotorType();
    void setMotorType(@NonNull E MotorType);
    int getCurrentCount();
    int getTargetCount();
    void setTargetCount(int Count);
    double getPower();
    void setPower(double Pwr);
    boolean isDirectionReversed();
    void setDirectionReversed(boolean isReversed);
    DcMotor.ZeroPowerBehavior getZeroPowerBehavior();
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior ZeroPwrBehavior);
    MovingType getCurrentMovingType();
    void setCurrentMovingType(MovingType movingType);
    String getMotorStatusString();
}
