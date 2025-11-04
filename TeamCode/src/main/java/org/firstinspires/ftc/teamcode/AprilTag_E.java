/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag_E {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private int goalTagId;

    public double distanceToGoal;

    private double bearing;

    private String ledColor;

    private double yaw;

    private boolean blueSide;
    private boolean redSide;
    private boolean PPG,PGP,GPP;

    private double goalBearingBlue, goalBearingRed;

    List<AprilTagDetection> currentDetections;

    @SuppressLint("DefaultLocale")
    public void scanField(Telemetry telemetry){

        currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        if(!currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                ledColor = "green";
                if (detection.metadata != null) {
                    if (detection.id == 20){
                        blueSide = true;
                        redSide = false;
                        goalBearingBlue = detection.ftcPose.bearing;
                        goalTagId = detection.id;
                    }
                    if (detection.id == 24){
                        blueSide = false;
                        redSide = true;
                        goalBearingRed = detection.ftcPose.bearing;
                        goalTagId = detection.id;
                    }
                    if (detection.id == 22){ //PGP
                        PPG = false;
                        GPP = false;
                        PGP = true;
                    }
                    if (detection.id == 23){ //PPG
                        PPG = true;
                        GPP = false;
                        PGP = false;
                    }
                    if (detection.id == 21){ //GPP
                        PPG = false;
                        GPP = true;
                        PGP = false;
                    }
                    telemetry.addLine(String.format("Bearing=%6.2f", detection.ftcPose.bearing));
                    telemetry.addLine(String.format("Range=%6.2f", detection.ftcPose.range));
                 }
            }

            telemetry.addLine(String.format("# AprilTags Detected: %d\n", currentDetections.size()));
            telemetry.addLine(String.format("GPP=%b, PGP=%b, PPG=%b\n", GPP, PGP, PPG));
            if (blueSide) {
                bearing = goalBearingBlue;
                telemetry.addLine(String.format("Blue  Goal:  Bearing=%6.2f", goalBearingBlue));
            }
            if (redSide) {
                bearing = goalBearingRed;
                telemetry.addLine(String.format("Red  Goal:  Bearing=%6.2f", goalBearingRed));
            }

            telemetry.update();
        } else {
            telemetry.addLine("No tags");
            bearing = 999;
            ledColor = "red";
        }
        //telemetry.addLine(String.format("Bearing=%6.2f", goalBearingBlue));
        telemetry.update();
    }

//    public void runInLoop(Telemetry telemetry) {

        // Wait for the DS start button to be touched.

//        telemetryAprilTag(telemetry);

        // Push telemetry to the Driver Station.
        // Save CPU resources; can resume streaming when needed.
//        if (gamepad1.dpad_down) {
//            visionPortal.stopStreaming();
//        } else if (gamepad1.dpad_up) {
//            visionPortal.resumeStreaming();
//        }

        // Share the CPU.

        // Save more CPU resources when camera is no longer needed.

//    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void closeAprilTag(){
        visionPortal.close();
    }
    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    public void runInLoop(Telemetry telemetry, boolean display) {

        currentDetections = aprilTag.getDetections();
        if (display) {
            telemetry.addData("# AprilTags Detected", currentDetections.size());
        }

        // Step through the list of detections and display info for each one.
        if(!currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == goalTagId) {
                    ledColor = "green";
                    if (detection.metadata != null) {
                        distanceToGoal = detection.ftcPose.y;
                        bearing = detection.ftcPose.bearing;
                        yaw = detection.ftcPose.yaw;
                        if (display) {
                            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                        }
                    } else {
                        if (display) {
                            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                        }
                    }
                }
                if (display) { telemetry.update(); };
            }
        } else{
            bearing = 999;
            ledColor = "red";
        }
    }   // end method telemetryAprilTag()

    public double getBearing() { return bearing; }

    public String getColor(){
        return ledColor;
    }

    public double getYaw() { return yaw; }
}