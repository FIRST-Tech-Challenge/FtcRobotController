package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private final CRServo intake;

    //servo bus current
    private final LynxModule exHub;
    private LynxGetADCCommand.Channel servoChannel;
    private LynxGetADCCommand servoCommand;
    private LynxGetADCResponse servoResponse;
    double servoBusCurrent;

    public IntakeSubsystem(HardwareMap hwMap) {
        intake = new CRServo(hwMap, "intake");
        intake.setInverted(true);
        exHub = hwMap.get(LynxModule.class, "Expansion Hub 2");
    }

    @Override
    public void periodic() {
        servoBusCurrent = calculateServoBusCurrent();
    }

    private double calculateServoBusCurrent() {
        servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        servoCommand = new LynxGetADCCommand(exHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            servoResponse = servoCommand.sendReceive();
            return servoResponse.getValue() / 1000.0;
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
}
