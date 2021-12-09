package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.groups.StandardCarfax;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.groups.StandardTank;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;

public class CarfaxDrivetrainLocation extends Location {

    private final StandardCarfax DRIVETRAIN;

    public CarfaxDrivetrainLocation(HardwareMap hardware) {
        StandardMotor rt = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_RIGHT_TOP_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD);
        StandardMotor lt = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_LEFT_TOP_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD);
        DRIVETRAIN = new StandardCarfax(rt, lt);
    }

    @Override
    public void stop() {
        DRIVETRAIN.stop();
    }

    @Override
    public boolean isInputLocation() {
        return true;
    }

    @Override
    public boolean isOutputLocation() {
        return false;
    }

    @Override
    public InteractionSurface getInternalInteractionSurface() {
        return DRIVETRAIN;
    }

}
