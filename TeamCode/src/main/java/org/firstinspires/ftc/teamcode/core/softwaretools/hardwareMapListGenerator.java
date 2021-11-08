package org.firstinspires.ftc.teamcode.core.softwaretools;
import android.os.Build;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;

public class hardwareMapListGenerator {
    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void listGen(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        List<String> list = new LinkedList<>();
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping : hardwareMap.allDeviceMappings) {
            for (Map.Entry<String, ? extends HardwareDevice> entry : deviceMapping.entrySet()) {
                list.add(entry.getKey());
            }
        }
        telemetry.addData("list", String.join("," ,list));
        telemetry.update();
    }
}
