package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


public class GoalTagLimelight {
    Limelight3A limelight;
    private double goalYaw; // inches
    private double goalRange; // in

    private int teamID;

    public boolean GPP = false; // id 21
    public boolean PGP = false; // id 22
    public boolean PPG = false; // id 23
    public boolean seeObelisk = false;
    private double tx;
    private double ty;
    private double camera_height = 15.625; // in
    private double target_height = 29.5; // in
    private double camera_angle = 0.04009; // radians old was 0.0418

    public boolean isDataCurrent;
    //Pipeline 5 is 20(blue) pipeline 1 is 24(red)

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(11);
        limelight.start(); // This tells Limelight to start looking!
    }

    public void readObelisk(Telemetry telemetry) {
        limelight.pipelineSwitch(6); //targets closest
        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            seeObelisk = false;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            if (tagId == 21) {
                telemetry.addData("Detected", "Tag ID 21");
                PGP = false;
                PPG = false;
                GPP = true;
                seeObelisk = true;
            } else if (tagId == 22) {
                telemetry.addData("Detected", "Tag ID 22");
                PGP = true;
                PPG = false;
                GPP = false;
                seeObelisk = true;
            } else if (tagId == 23) {
                telemetry.addData("Detected", "Tag ID 23");
                PGP = false;
                PPG = true;
                GPP = false;
                seeObelisk = true;
            }
        }
    }
    public void process(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)

            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                telemetry.addData("botx", x);
                double y = botpose.getPosition().y;
                telemetry.addData("boty", y);

                goalYaw = botpose.getOrientation().getYaw();
                goalRange = (target_height - camera_height) / (Math.tan(Math.toRadians(ty)+camera_angle));
                double denom = Math.tan(Math.toRadians(camera_angle + ty));
                telemetry.addData("denom", denom);

                isDataCurrent = true;

                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            } else {
                isDataCurrent = false;
            }

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("pipeline", result.getPipelineIndex());
            telemetry.addData("limelight Range", goalRange);
            telemetry.addData("Target Area", ta);
        } else {
            isDataCurrent = false;
            telemetry.addData("Limelight", "No Targets");
        }
    }

    public void setTeam(int id) {
        if (id == 20) {
            limelight.pipelineSwitch(5);
            teamID = 20;
        } else if (id == 24) {
            limelight.pipelineSwitch(1);
            teamID = 24;
        }
    }

    public String getObelisk() {
        if (PGP) {
            return "PGP";
        } else if (GPP) {
            return "GPP";
        } else if (PPG) {
            return "PPG";
        } else {
            return "No Tag Detected";
        }
    }
    public double getRange() {
        return goalRange;
    }
    public double getTx() {
        return tx;
    }
    public int getID() {
        return teamID;
    }
}
