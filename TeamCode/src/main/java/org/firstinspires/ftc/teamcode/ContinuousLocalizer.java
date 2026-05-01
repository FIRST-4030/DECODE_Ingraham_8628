package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ContinuousLocalizer {
    private static double COOLDOWN_MS = 500;

    private ElapsedTime cooldown = new ElapsedTime();
    private Follower follower;
    private Limelight limelight;

    public ContinuousLocalizer(Follower followerValue, Limelight limelightValue) {
        follower = followerValue;
        limelight = limelightValue;
    }

    public void update() {
        if (cooldown.milliseconds() < COOLDOWN_MS) {
            return;
        }

        limelight.processRobotPoseMt1();

        if (limelight.isDataCurrent) {
            limelight.getX();
            limelight.getY();
        }
    }
}
