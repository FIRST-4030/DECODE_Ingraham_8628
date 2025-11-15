package org.firstinspires.ftc.teamcode.OpModes;

public class Blackboard_ETJ {
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
