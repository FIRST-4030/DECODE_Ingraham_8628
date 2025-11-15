package org.firstinspires.ftc.teamcode;

public class Blackboard {
    public enum Alliance {
        RED,
        BLUE,
        UNKNOWN,
    }
    public static Alliance alliance = Alliance.UNKNOWN;

    public static String getAllianceAsString() {
        switch (alliance) {
            case RED:
                return "Red";
            case BLUE:
                return "Blue";
            case UNKNOWN:
                return "Unknown";
        }
        return "Undefined";
    }
}
