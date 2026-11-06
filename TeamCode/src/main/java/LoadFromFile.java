//NRuke
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "load from file")
@Disabled
public class LoadFromFile extends OpMode {
        private List<Integer> routine;

        @Override
    public void init() {
         routine = new ArrayList<>();

         File closeAuto = AppUtil.getInstance().getSettingsFile("Saving_and_Loading_on_text_file");
         String[] types = ReadWriteFile.readFile(closeAuto).trim().split(",");

         for (String type : types) {
             if (!type.isEmpty()) {
                 routine.add(Integer.parseInt(type));
             }
         }
        }


    @Override
    public void loop() {
        telemetry.addData("routine", routine);

    }
}
