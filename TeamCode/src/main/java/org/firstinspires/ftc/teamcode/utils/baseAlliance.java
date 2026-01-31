package org.firstinspires.ftc.teamcode.utils;

public class baseAlliance {
    public enum color {
        RED,
        BLUE
    }
    public enum base {
        CLOSE,
        FAR
    }
    public color color;
    public base base;
    public baseAlliance(color color, base base) {
        this.color = color;
        this.base = base;
    }
}
