package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import com.qualcomm.robotcore.util.RollingAverage;
import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final CRServo intake;

    //servo bus current
    private final LynxModule exHub;
    private final RollingAverage averageCurrent;
    double servoBusCurrent = 0;

    public IntakeSubsystem(HardwareMap hwMap) {
        intake = new CRServo(hwMap, "intake");
        intake.setInverted(true);
        exHub = hwMap.get(LynxModule.class, "Expansion Hub 2");
        averageCurrent = new RollingAverage(Constants.INTAKE_AVG_LENGTH);
    }

    @Override
    public void periodic() {
        servoBusCurrent = calculateServoBusCurrent();
        averageCurrent.addNumber((int) servoBusCurrent);
    }

    private double calculateServoBusCurrent() {
        LynxGetADCCommand.Channel servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        LynxGetADCCommand servoCommand = new LynxGetADCCommand(exHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse servoResponse = servoCommand.sendReceive();
            return servoResponse.getValue();
        } catch (InterruptedException | RuntimeException | LynxNackException ignored) {
        }
        return 999;
    }

    public void setPower(double power) {
        intake.set(power);
    }

    public double getPower() {
        return intake.get();
    }

    public double getServoBusCurrent() {
        return servoBusCurrent;
    }

    public double getAverageCurrent() {
        return averageCurrent.getAverage();
    }
}
