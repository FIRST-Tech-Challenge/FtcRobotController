package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class RotatorSubsystem extends SubsystemBase {

    private final ServoImplEx rotator;

    //servo bus current
    private final LynxModule cHub;
    private final RollingAverage averageCurrent;
    double servoBusCurrent = 0;

    public RotatorSubsystem(final HardwareMap hwMap) {
        rotator = hwMap.get(ServoImplEx.class, "rotator");
        rotator.setDirection(Servo.Direction.REVERSE);
        cHub = hwMap.get(LynxModule.class, "Control Hub");
        averageCurrent = new RollingAverage(Constants.ROTATOR_AVG_LENGTH);
    }

    @Override
    public void periodic() {
        servoBusCurrent = calculateServoBusCurrent();
        averageCurrent.addNumber((int) servoBusCurrent);
    }

    private double calculateServoBusCurrent() {
        LynxGetADCCommand.Channel servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        LynxGetADCCommand servoCommand = new LynxGetADCCommand(cHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse servoResponse = servoCommand.sendReceive();
            return servoResponse.getValue();
        } catch (InterruptedException | RuntimeException | LynxNackException ignored) {
        }
        return 999;
    }

    public void enable() {
        rotator.setPwmEnable();
    }

    public void disable() {
        rotator.setPwmDisable();
    }

    public void setPosition(double pos) {
        rotator.setPosition(pos);
    }

    public void setPWMRange(PwmControl.PwmRange range) {
        rotator.setPwmRange(range);
    }

    public double getPosition() {
        return rotator.getPosition();
    }

    public double[] getPWMRange() {
        PwmControl.PwmRange range = rotator.getPwmRange();
        return new double[]{range.usFrame, range.usPulseLower, range.usPulseUpper};
    }

    public double getAverageCurrent() {
        return averageCurrent.getAverage();
    }
}
