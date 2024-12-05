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
                "Sample{x=7.99, y=-2.61, d=3.591, a=-33, whr=1.51, c=Red}");
    }
    @org.junit.jupiter.api.Test
    void testValid2() throws IOException, JSONException {
        testWithJsonFile("valid_2.json",
                "Sample{x=-11.1, y=-4.85, d=3.732, a=-76, whr=3.59, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testValid3() throws IOException, JSONException {
        testWithJsonFile("valid_3.json",
                "Sample{x=2.34, y=4.75, d=3.266, a=33, whr=1.62, c=Blue}");
    }
    @org.junit.jupiter.api.Test
    void testValid4() throws IOException, JSONException {
        testWithJsonFile("valid_4.json",
                "Sample{x=1.08, y=1.73, d=3.006, a=4, whr=1.18, c=Unknown}");
    }
    @org.junit.jupiter.api.Test
    void testSample1() throws IOException, JSONException {
        testWithJsonFile("sample_1.json",
                "Sample{x=-5.29, y=-1.7, d=1.78, a=-71, whr=2.04, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testSpicimen1() throws IOException, JSONException {
        testWithJsonFile("specimen_1.json",
                "Sample{x=0.0, y=-4.3, d=1.574, a=0, whr=1.95, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testSpicimen2() throws IOException, JSONException {
        testWithJsonFile("specimen_2.json",
                "Sample{x=-9.53, y=0.47, d=1.397, a=34, whr=2.08, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testSpicimen3() throws IOException, JSONException {
        testWithJsonFile("specimen_3.json",
                "Sample{x=-4.19, y=-6.11, d=1.319, a=89, whr=1.77, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testSpicimen4() throws IOException, JSONException {
        testWithJsonFile("specimen_4.json",
                "Sample{x=5.58, y=-3.01, d=1.296, a=-87, whr=1.8, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testSpicimen5() throws IOException, JSONException {
        testWithJsonFile("specimen_5.json",
                "Sample{x=-18.91, y=-1.74, d=1.448, a=-17, whr=2.1, c=Shared}");
    }
}