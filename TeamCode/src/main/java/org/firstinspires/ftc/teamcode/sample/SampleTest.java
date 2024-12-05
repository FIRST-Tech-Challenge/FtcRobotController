package org.firstinspires.ftc.teamcode.sample;

import static org.junit.jupiter.api.Assertions.*;

import android.annotation.SuppressLint;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.firebase.crashlytics.buildtools.reloc.org.apache.commons.io.IOUtils;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Paths;


class SampleTest {

    class MockLLResult extends LLResult {
        public MockLLResult(JSONObject json) throws JSONException {
            super(json);
        }
    }
    public static JSONObject parseJSONFile(String filename) throws JSONException, IOException {
        @SuppressLint("NewApi") String content = new String(Files.readAllBytes(Paths.get(filename)));
        return new JSONObject(content);
    }

    public void testWithJsonFile(String filename, String expected) throws IOException, JSONException {
        // Read JSON file
        JSONObject json = parseJSONFile("src/main/java/org/firstinspires/ftc/teamcode/sample/test_data/" + filename);
        MockLLResult llResult = new MockLLResult(json);

        // Create Sample instance
        Sample sample = new Sample(llResult);

        // Perform assertions
        assertEquals(expected, sample.toString());
    }

    @org.junit.jupiter.api.Test
    void testInvalidSmallTa() throws IOException, JSONException {
        testWithJsonFile("invalid_small_ta.json",
                "Invalid LLResult with error code -10032");
    }
    @org.junit.jupiter.api.Test
    void testValid1() throws IOException, JSONException {
        testWithJsonFile("valid_1.json",
                "Sample{x=7.79, y=-2.81, d=1.14, a=-33, whr=1.51, c=Red}");
    }
    @org.junit.jupiter.api.Test
    void testValid2() throws IOException, JSONException {
        testWithJsonFile("valid_2.json",
                "Sample{x=-11.3, y=-5.05, d=1.281, a=-76, whr=3.59, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testValid3() throws IOException, JSONException {
        testWithJsonFile("valid_3.json",
                "Sample{x=2.14, y=4.55, d=0.814, a=33, whr=1.62, c=Blue}");
    }
    @org.junit.jupiter.api.Test
    void testValid4() throws IOException, JSONException {
        testWithJsonFile("valid_4.json",
                "Sample{x=0.88, y=1.53, d=0.555, a=4, whr=1.18, c=Unknown}");
    }
}