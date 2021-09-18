package com.bravenatorsrobotics.core;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * Created by dmill on 10/28/2017.
 */




class Option
{
    private String name;
    private String[] choices;
    public int choiceIndex = 0;

    public Option(String name, String[] choices)
    {
        this.name = name;
        this.choices = choices;
    }
    public Option(String name, String[] choices, String initialChoice)
    {
        this.name = name;
        this.choices = choices;

        for(int i = 0;i<choices.length;i++){
            if(choices[i] == initialChoice) {
                this.choiceIndex = i;
                break;
            }
        }

    }



    public Option(String name, double max, double min, double inc)
    {
        this.name = name;

        int size = (int) ((max-min)/inc);
        choices = new String[size];
        for (int i = 0; i < size; i++)
        {
            choices[i] = String.valueOf(inc*i);
        }

        choiceIndex = 0;           //fix this rock hard code
    }


    public Option(String name, int max, int min, int inc, int start){
        this.name = name;

        int size = (int) (((max+inc)-min)/inc);
        choices = new String[size];
        for (int i = 0; i < size; i++)
        {
            double val = min + (inc*i);
            choices[i] = String.valueOf(val);
            if(Math.abs(val - start) < 0.001) {
                choiceIndex = i;
            }
        }
    }
    public Option(String name, double max, double min, double inc, double start)
    {
        this.name = name;

        int size = (int) (((max+inc)-min)/inc);
        choices = new String[size];
        for (int i = 0; i < size; i++)
        {
            double val = min + (inc*i);
            choices[i] = String.valueOf(val);
            if(Math.abs(val - start) < 0.001) {
                choiceIndex = i;
            }
        }
    }

    public String getName()
    {
        return name;
    }
    public String[] getChoices()
    {
        return choices;
    }
    public String getCurrentChoice()
    {
        return choices[choiceIndex];
    }
    public int getNumChoices()
    {
        return choices.length;
    }
}

public class SimpleMenu {
    private Telemetry telemetry;
    private String menuTitle;
    private Gamepad gamepad;
    int currentOption = 0;
    private boolean[] buttonStates = new boolean[4];
    private ArrayList<Option> options;

    public SimpleMenu(String menuTitle) {
        this.menuTitle = menuTitle;
        options = new ArrayList();
    }


    public void setTelemetry(Telemetry t)
    {
        this.telemetry = t;
    }
    public void setGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
    }

    public void clearOptions()
    {
        options.clear();
    }
    public void loadFrom(ArrayList<Option>options)
    {
        this.options = options;
    }
    public ArrayList<Option> getOptionsConfig()
    {
        return options;
    }

    public void addOption(String option, String[] choices)
    {
        this.options.add(new Option(option, choices));
    }

    public void addOption(String option, String[] choices, String initialChoice)
    {
        this.options.add(new Option(option, choices, initialChoice));
    }

    public <T extends Enum<T>> void addOption(String option, Class<T> enumClass, T initialChoice) {

        ArrayList<String> options = new ArrayList<>();

        for(T constant : enumClass.getEnumConstants()) {
            options.add(constant.toString());
        }

        this.options.add(new Option(option, options.toArray(new String[0]), initialChoice.toString()));

    }

    public void addBooleanOption(String option, boolean initialChoice)
    {
        this.options.add(new Option(option, new String[] { "YES", "NO"}, initialChoice ? "YES" : "NO"));
    }

    public void addOption(String option, double max, double min, double inc)
    {
        this.options.add(new Option(option, max, min, inc));
    }
    public void addOption(String option, double max, double min, double inc, double start)
    {
        this.options.add(new Option(option, max, min, inc, start));
    }
    public void addOption(String option, int max, int min, int inc, int start)
    {
        this.options.add(new Option(option, (int)max, (int)min, inc, start));
    }
    public String getCurrentChoiceOf(String option) {
        for (Option o : this.options) {
            if (!o.getName().equals(option)) continue;
            return o.getCurrentChoice();
        }
        return "ERROR - NO OPTION BY THAT NAME";
    }

    public void displayMenu() {
        Option o;
        int loops = 1;

        if(this.gamepad.right_bumper) {
            loops *= 10;
        }
        if(this.gamepad.left_bumper) {
            loops *= 10;
        }


        if (this.checkButton(this.gamepad.dpad_down, 1)) {
            this.currentOption = this.currentOption < this.options.size() - 1 ? ++this.currentOption : 0;
        } else if (this.checkButton(this.gamepad.dpad_up, 0)) {
            this.currentOption = this.currentOption > 0 ? --this.currentOption : this.options.size() - 1;
        } else if (this.checkButton(this.gamepad.dpad_left || this.gamepad.b, 2)) {
            o = this.options.get(this.currentOption);
            for(int i = 0;i<loops;i++) {
                o.choiceIndex = o.choiceIndex > 0 ? --o.choiceIndex : o.getNumChoices() - 1;
            }
        } else if (this.checkButton(this.gamepad.dpad_right || this.gamepad.a, 3)) {
            o = this.options.get(this.currentOption);
            for(int i = 0;i<loops;i++) {
                o.choiceIndex = o.choiceIndex < o.getNumChoices() - 1 ? ++o.choiceIndex : 0;
            }
        }
        this.telemetry.addData("Menu", (Object)this.menuTitle);
        int count = 0;
        for (Option o2 : this.options) {
            if (this.currentOption == count) {
                this.telemetry.addData(">> " + o2.getName(), o2.getCurrentChoice());
            } else {
                String name = o2.getName();

                this.telemetry.addData(name, o2.getCurrentChoice());
            }
            ++count;
        }
        this.telemetry.update();
    }

    public void displayConfig(Telemetry telemetry)
    {
        for (Option o2 : this.options)
        {
            telemetry.addData(o2.getName(), o2.getCurrentChoice());
            telemetry.update();
        }
    }

    private boolean checkButton(boolean b, int i) {
        if (b != this.buttonStates[i]) {
            this.buttonStates[i] = !this.buttonStates[i];
            return this.buttonStates[i];
        }
        return false;
    }
}
