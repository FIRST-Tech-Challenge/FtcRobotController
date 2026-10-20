package org.firstinspires.ftc.teamcode.opMode;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.InOrder;

/** A motor set up for the pod test must be safe to push by hand. */
@DisplayName("Test: Odometry Pods")
class OdometryPodTestTest {

    @Test
    @DisplayName("a motor set up to be pushed has no power, no brake, and a count of 0")
    void pushable() {
        DcMotorEx motor = mock(DcMotorEx.class);
        OdometryPodTest.SetUpToBePushed(motor);
        verify(motor).setPower(0);
        verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        verify(motor, never()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset first, then leave the motor in a mode where the count keeps changing.
        InOrder order = inOrder(motor);
        order.verify(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        order.verify(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Test
    @DisplayName("never asks for any power but 0")
    void noPower() {
        DcMotorEx motor = mock(DcMotorEx.class);
        OdometryPodTest.SetUpToBePushed(motor);
        verify(motor).setPower(anyDouble());
        verify(motor).setPower(0);
    }
}
