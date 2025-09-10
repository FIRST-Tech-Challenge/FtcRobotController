package org.firstinspires.ftc.teamcode.PathingTool;

import android.content.Context;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

public class PathLoading {

    private static JSONObject jsonPathData;

    public PathLoading(Context context, String pathFileName) {
        loadJSONFromAsset(context, pathFileName);
    }

    private void loadJSONFromAsset(Context context, String pathFileName) {
        try {
            InputStream inputStream = context.getAssets().open(pathFileName);
            BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }
            reader.close();
            inputStream.close();

            jsonPathData = new JSONObject(sb.toString());
        } catch (IOException | JSONException e) {
            e.printStackTrace();
        }
    }

    public static JSONObject getJsonPathData() {
        return jsonPathData;
    }
}
