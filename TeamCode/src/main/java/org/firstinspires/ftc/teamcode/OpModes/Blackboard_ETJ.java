package org.firstinspires.ftc.teamcode.OpModes;

public class Blackboard_ETJ {
    public enum Alliance {
        RED,
        BLUE,
        UNKNOWN,
    }
    public static Alliance alliance = Alliance.UNKNOWN;

    public static String getAllianceAsString() {
        if (alliance == Alliance.RED) {
            return "Red";
        } else if (alliance == Alliance.BLUE) {
            return "Blue";
        } else if (alliance == Alliance.UNKNOWN) {
            return "Unknown";
        }
        return "Undefined";
    }
}
