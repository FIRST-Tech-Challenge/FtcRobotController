package org.firstinspires.ftc.teamcode.Util;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;


/**
 * IniFile and CFG file parser.
 */
public class IniFile {

    ArrayList<ArrayList<String>> parseResult = new ArrayList<>(1);

    public IniFile(String path) {
        try {
            load(path);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void load(String path) throws IOException {
        int lineNumber = 0;
        String section = "";
        File file = new File(path);
        FileReader fr = new FileReader(file);
        char[] a = new char[50];
        fr.read(a);   // reads the content to the array
        StringBuilder output = new StringBuilder();

        for (char c : a)
            output.append(c);   // prints the characters one by one
        fr.close();

        String[] outputSplit = output.toString().split("\n");
        for (String line : outputSplit) {
            String new_line = line.replace(" ", "");
            if ((new_line.charAt(0) != '#') && (new_line.charAt(0) != '[')) {
                String[] key_and_value = new_line.split("=");
                if (key_and_value.length == 2) {
                    ArrayList<String> toAdd = new ArrayList<>(3);
                    toAdd.add(section);
                    toAdd.add(key_and_value[0]);
                    toAdd.add(key_and_value[1]);
                    parseResult.add(toAdd);
                    lineNumber++;
                }
            } else if (new_line.charAt(0) == '[') {
                section = new_line.substring(1, new_line.length() - 2);
            }
        }
    }

    public ArrayList<ArrayList<String>> getParseResult() {
        return this.parseResult;
    }

    public ArrayList<String> get(int place) {
        return parseResult.get(place);
    }

    public String getValue(int place) {
        return parseResult.get(place).get(2);
    }

    public String getValue(String key) {
        for (ArrayList<String> line : parseResult) {
            if ((line.get(1)).equals(key)) {
                return line.get(2);
            }
        }
        throw new NoSuchFieldError(key + " is not a key.");
    }

    public String getSection(int place) {
        return parseResult.get(place).get(0);
    }
}
