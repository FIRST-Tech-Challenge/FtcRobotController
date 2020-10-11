package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

@com.qualcomm.robotcore.hardware.configuration.annotations.MotorType(ticksPerRev = 145.6, gearing = 5.2, maxRPM = 1150, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "goBILDA5202Series1150RPMMotor", name = "GoBILDA 5202 series 1150RPM Motor", builtIn = true)
@DistributorInfo(distributor = "goBILDA_distributor", model = "goBILDA-5202", url = "https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motors/")
public class GoBilda5202Series1150RPMMotor implements MotorType {
    @Override
    public String getMotorName() {
        return "GoBlida 5202 Series 1150RPM Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 145.6;
    }

    @Override
    public double getRevPerSec() {
        return 19.167; //1150 rpm/60
    }
}
