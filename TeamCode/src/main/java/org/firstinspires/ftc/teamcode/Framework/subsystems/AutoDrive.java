package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class AutoDrive extends SubsystemBase {

    private SampleMecanumDrive drive;

    public AutoDrive(HardwareMap hw){
        this.drive = new SampleMecanumDrive(hw);
    }

    public SampleMecanumDrive getDrive(){
        return this.drive;
    }

}
