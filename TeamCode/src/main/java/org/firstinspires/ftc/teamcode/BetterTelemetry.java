package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class BetterTelemetry extends TelemetryImpl {
    public static final Color DEFAULT_COLOR = Color.WHITE;
    public static final int DEFAULT_FONT_SIZE = 4;
    public static final String DEFAULT_FONT = "Courier New";
    public enum Color{
        RED("FF2222"), ORANGE("EA5D00"), YELLOW("FFBF00"), LIME("69FF00"),
        GREEN("00D318"), CYAN("00E5E5"), BLUE("1E39CE"), PURPLE("5A00E2"),
        MAGENTA("BE00FF"), PINK("FF3ADC"), BLACK("000000"), WHITE("FFFFFF"),
        LIGHT_GRAY("808080"), DARK_GRAY("404040"), NO_COLOR("FFFFFF");
        String hexValue;
        Color(String hex){
            hexValue = hex;
        }
        String getHexValue(){
            return hexValue;
        }

    }
    public static class Format {
        public Color color = Color.NO_COLOR;
        public int fontSize = -1;
        public String font = "none";
        public boolean bold, underline, italic, strikeThrough;
        private StringBuilder builder;
        private Format(){

        }
        public Format makeDefault() {
            color = DEFAULT_COLOR;
            fontSize = DEFAULT_FONT_SIZE;
            font = DEFAULT_FONT;
            return this;
        }
        public Format color(Color c){
            color = c;
            return this;
        }
        public Format fontSize(int s){
            fontSize = s;
            return this;
        }
        public Format font(String s){
            font = s;
            return this;
        }
        public Format bold(boolean b){
            bold = b;
            return this;
        }
        public Format italic(boolean b){
            italic = b;
            return this;
        }
        public Format underline(boolean b){
            underline = b;
            return this;
        }
        public Format strikeThrough(boolean b){
            strikeThrough = b;
            return this;
        }
        private void addTag(String tagName) {
            builder.insert(0, "<" + tagName + ">");
            builder.append("</" + tagName + ">");
        }

        public String format(String text) {
            builder = new StringBuilder();
            builder.append(text);
            if (bold)
                addTag("b");
            if (italic)
                addTag("i");
            if (underline)
                addTag("u");
            if (strikeThrough)
                addTag("s");
            return builder.toString();
        }
    }
    public String getFontFormatted(String f, int s, Color c){
        return "<font "+(s == -1 ? "" : "size="+s+" ")+(f.equals("none") ? "" : "font=\""+f+"\" ")
                +(c == Color.NO_COLOR ? "" : "color=#"+c.getHexValue()+" ")+">";
    }

    public String getFormatted(Format t, String text){
        return getFontFormatted(t.font, t.fontSize, t.color)+t.format(text)+"</font>";
    }

    public String getFormatted(String text){
        return getFormatted(textFormat, text);
    }


    public Format textFormat = new Format().makeDefault();
    public BetterTelemetry(OpMode opMode) {
        super(opMode);
        opMode.telemetry.setDisplayFormat(DisplayFormat.HTML);

    }
    public BetterTelemetry(OpMode opMode, Format format) {
        super(opMode);
        opMode.telemetry.setDisplayFormat(DisplayFormat.HTML);
        textFormat = format;
    }
    public BetterTelemetry setFormat(Format format){
        opMode.telemetry.setDisplayFormat(DisplayFormat.HTML);
        textFormat = format;
        return this;
    }

    public static Format format(){
        return new Format();
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return super.addData(getFormatted(caption), getFormatted(format), args);
    }

    @Override
    public Item addData(String caption, Object value) {
        return super.addData(getFormatted(caption), getFormatted(value+""));
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return super.addData(getFormatted(caption), getFormatted(format), valueProducer);
    }

    @Override
    public Line addLine(String lineCaption) {
        return super.addLine(getFormatted(lineCaption));
    }


    public Item addData(Format f, String caption, String format, Object... args) {
        return super.addData(getFormatted(f, caption), getFormatted(f, format), args);
    }
    public Item addData(Format f1, String caption, Format f2, String format, Object... args) {
        return super.addData(getFormatted(f1, caption), getFormatted(f2, format), args);
    }


    public Item addData(Format f, String caption, Object value) {
        return super.addData(getFormatted(f, caption), getFormatted(f, value+""));
    }
    public Item addData(Format f1, String caption, Format f2, Object value) {
        return super.addData(getFormatted(f1, caption), getFormatted(f2, value+""));
    }


    public <T> Item addData(Format f, String caption, String format, Func<T> valueProducer) {
        return super.addData(getFormatted(f, caption), getFormatted(f, format), valueProducer);
    }

    public <T> Item addData(Format f1, String caption, Format f2, String format, Func<T> valueProducer) {
        return super.addData(getFormatted(f1, caption), getFormatted(f2, format), valueProducer);
    }

    public Line addLine(Format f, String lineCaption) {
        return super.addLine(getFormatted(f, lineCaption));
    }
}
