package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.File;
import java.io.FileOutputStream;
import java.util.Calendar;


public class FileUtils {

  /**
   * Saves a bitmap to the phone. Sets the file name using time & date information.
   * Saves can be found in the downloads folder of the phone.
   */
  public static void saveImage(Bitmap bitmap, Constants.SamplingLocation location) {
    String loc = location == null ? "" : location.name() + "_";
    Calendar now = Calendar.getInstance();
    String filePath = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString();
    String fileName = "BotImg_" + loc + now.get(Calendar.DAY_OF_MONTH) + "_" + now.get(Calendar.HOUR_OF_DAY) + "_" + now.get(Calendar.MINUTE) + "_" + now.get(Calendar.SECOND) + now.get(Calendar.MILLISECOND) + ".jpg";
    File img = new File(filePath, fileName);
    if (img.exists())
      img.delete();
    try {
      FileOutputStream out = new FileOutputStream(img);
      bitmap.compress(Bitmap.CompressFormat.JPEG, 100, out);
      out.flush();
      out.close();
    } catch (Exception e) {
      throw new Warning(e.getMessage());
    }
  }

}
