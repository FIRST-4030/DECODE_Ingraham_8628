package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Limelight {

    private Telemetry telemetry;

    public Limelight3A limelight;
    private double goalYaw; // inches
    private double goalRange; // in
    private double blueSideX, blueSideY, redSideX, redSideY;
    private double blueSideXMT2, blueSideYMT2, redSideXMT2, redSideYMT2;

    private int goalTagId;
    private int teamID;
    private static double METERS_TO_INCHES = 39.3701;

    private double tx, ty, x, y;

    private boolean PPG,PGP,GPP;
    private boolean targetInView = false;
    public boolean seeObelisk = false;
    public boolean isDataCurrent;

    private final double camera_height = 16.75; // in
    private final double target_height = 29.5; // in
    private double camera_angle = -0.042; // Using LimelightAngleSetter

    IMU imu;
//
//    @SuppressLint("DefaultLocale")
//    public void getTagLocations(String color) {
//        LLResult result;
////        YawPitchRollAngles orientation;
//
//        if (color.equals("Red")) {
//            limelight.pipelineSwitch(1);
//
////            orientation = imu.getRobotYawPitchRollAngles();
////            limelight.updateRobotOrientation(orientation.getYaw());
//
//            result = limelight.getLatestResult();
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                int tagId = fiducial.getFiducialId();
//                if (tagId==24) {
//                    Pose3D botposeRedMT2 = result.getBotpose_MT2();
//                    redSideXMT2 = botposeRedMT2.getPosition().x;
//                    redSideYMT2 = botposeRedMT2.getPosition().y;
//                    Pose3D botposeRed = result.getBotpose();
//                    redSideX = botposeRed.getPosition().x;
//                    redSideY = botposeRed.getPosition().y;
//                }
//            }
//        }
//
//        if (color.equals("Blue")) {
//            limelight.pipelineSwitch(5);
//
////            orientation = imu.getRobotYawPitchRollAngles();
////            limelight.updateRobotOrientation(orientation.getYaw());
//
//            result = limelight.getLatestResult();
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                int tagId = fiducial.getFiducialId();
//                if (tagId==20) {
//                    Pose3D botposeBlueMT2 = result.getBotpose_MT2();
//                    blueSideXMT2 = botposeBlueMT2.getPosition().x;
//                    blueSideYMT2 = botposeBlueMT2.getPosition().y;
//                    Pose3D botposeBlue = result.getBotpose();
//                    blueSideX = botposeBlue.getPosition().x;
//                    blueSideY = botposeBlue.getPosition().y;
//                }
//            }
//        }
//    }

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

    public void init(HardwareMap hardwareMap,Telemetry telemetry) {
        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(11);
        limelight.start(); // This tells Limelight to start looking!
    }

    public boolean process(Telemetry telemetry) {

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();

            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)

            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            if (botPose != null) {

                goalYaw = botPose.getOrientation().getYaw();
                goalRange = (target_height - camera_height) / (Math.tan(Math.toRadians(ty)+camera_angle));

                isDataCurrent = true;
            } else {
                isDataCurrent = false;
            }

            telemetry.addData("pipeline", result.getPipelineIndex());
            telemetry.addData("limelight Range", goalRange);
        } else {
            isDataCurrent = false;
            telemetry.addData("Limelight", "No Targets");
        }
        return isDataCurrent;
    }

    public void processRobotPose() {
        limelight.pipelineSwitch(6); // obelisk
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();
            x = botPose.getPosition().x*METERS_TO_INCHES + 72;
            y = botPose.getPosition().y*METERS_TO_INCHES + 72;
        }
    }

    @SuppressLint("DefaultLocale")
    public void readObelisk() {

        limelight.pipelineSwitch(6); //targets closest

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            seeObelisk = false;
        } else {

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int tagId = fiducial.getFiducialId();

                if (tagId == 22) { //PGP
                    PPG = false;
                    GPP = false;
                    PGP = true;
                    seeObelisk = true;
                } else if (tagId == 23) { //PPG
                    PPG = true;
                    GPP = false;
                    PGP = false;
                    seeObelisk = true;
                } else if (tagId == 21) { //GPP
                    PPG = false;
                    GPP = true;
                    PGP = false;
                    seeObelisk = true;
                }
            }
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

    public void setCameraAngle(double angle) {
        this.camera_angle = angle;
    }

    public int getPipeline() { return limelight.getStatus().getPipelineIndex(); }
    public int getTeam() { return teamID; }

    public double getCameraAngle() { return camera_angle; }
    public double getX() { return x; }
    public double getY() { return y; }
    public double getYaw() { return goalYaw; }
    public double getRange() { return goalRange; }
    public double getGoalTagId() { return goalTagId; }

    public void setGoalTagID(int value) { goalTagId = value; }
}