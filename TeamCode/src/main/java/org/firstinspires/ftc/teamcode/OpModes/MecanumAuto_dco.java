/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTags_dco;
import org.firstinspires.ftc.teamcode.Datalogger;

@Autonomous(name="DCO: Mecanum Auto", group="DCO")
public class MecanumAuto_dco extends LinearOpMode {

    public static boolean logData = true;
    private final int NEVERREST_TICKS_PER_REV = 1120;
    private final double DIAMETER_GREY = 3.5;
    private final double DIAMETER_GOBILDA = 100. / 10 / 2.54;
    private int distance = 0;
    private double diameter = 0;
    private String wheels = "";
    public static int decimation = 1;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    int eee;

    private String allianceSide = "";
    private int allianceId = 0;

    DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    IMU imu;
    AprilTags_dco aprilTags;
    Datalog datalog;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /*
         * Code that runs ONCE when the driver hits START
         *    Tasks to include are,
         *       - Initialize April tags
         *       - Initialize any time parameters
         */

        // Initialize the datalog
        if (logData) {
            datalog = new Datalog(String.format("AprilTagLog"));
        }

        runtime.reset();

        aprilTags = new AprilTags_dco();
        aprilTags.initAprilTag(hardwareMap, telemetry, imu);

        do {
            aprilTags.scanForObelisk();
        } while (opModeInInit());
        //waitForStart();

        rotateTo(aprilTags.getBearing());
        DriveForwardForTime(0.75, 1.0);

//        /* Data log
//         * Note: The order in which we set datalog fields does *not* matter!
//         *       Order is configured inside the Datalog class constructor.
//         */
//        if (logData) {
//            datalog.loopCounter.set(i);
//            datalog.tagCounter.set(aprilTags.getCurrentPositionCounter());
//            datalog.inLoopCounter.set(aprilTags.getRunInLoopCounter());
//            datalog.runTime.set(runtime.seconds());
//            datalog.bearing.set(bearing);
//            datalog.power.set(power);
//            datalog.error.set(error);
//            datalog.writeLine();
//        }
//        i++;
    }

    public void DriveForwardDistance(double power, int distance) {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(distance);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(power);

        while (frontLeftDrive.isBusy() &&
               frontRightDrive.isBusy() &&
               backLeftDrive.isBusy() &&
               backRightDrive.isBusy()) {
            // Just wait until distance is achieved
        }

        StopDriving();
    }

    public void DriveForward(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void DriveForwardForTime(double power, double timeInSec) {

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (timer.seconds() < timeInSec) {
            frontLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            backLeftDrive.setPower(power);
            backRightDrive.setPower(power);
        }

        // Stop motors
        StopDriving();
    }

    public void StopDriving() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public int CalculateDistanceInTicks(double travelDistance,double radiusInInches) {
        /*
         * Calculate the number of wheel rotations you need to go the distance
         */
        double circumference = 2*Math.PI*radiusInInches;
        double rotations = travelDistance / circumference;
        /*
         * Convert arc length to ticks
         */
        int ticks = (int) (rotations * NEVERREST_TICKS_PER_REV);
        eee = ticks;
        return ticks;
    }

    private void rotateTo(double targetAngle) {
        double Kp = 0.082;  // Proportional gain (tune this)
        double Kd = 0.005;  // derivative gain
        double minPower = 0.3;
        double maxPower = 0.5;
        double tolerance = 1.0; // degrees
        double lastError = 0;
        double derivative;
        double currentAngle, error, turnPower;

        long lastTime = System.nanoTime();

        while (true) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            error = targetAngle - currentAngle;
            error = (error + 540) % 360 - 180; // Wrap error to [-180, 180] range

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            derivative = (error - lastError) / deltaTime;
            lastError = error;

            if (Math.abs(error) < tolerance) break;

            turnPower = Kp * error + Kd *derivative;

            // Enforce minimum power
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }
            // Clamp maximum power
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            telemetry.addData("Target (deg)", "%.2f", targetAngle);
            telemetry.addData("Current (deg)", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Turn Power", "%.2f", turnPower);
            telemetry.update();

            frontLeftDrive.setPower(-turnPower);
            backLeftDrive.setPower(-turnPower);
            frontRightDrive.setPower(turnPower);
            backRightDrive.setPower(turnPower);
        }

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
//        sleep(100);

//        telemetry.addData("Target (deg)", "%.2f", targetAngle);
//        telemetry.addData("Current (deg)", "%.2f", currentAngle);
//        telemetry.addData("Error", "%.2f", error);
//        telemetry.addData("Turn Power", "%.2f", turnPower);
//        telemetry.addLine("Aligned with AprilTag");
//        telemetry.update();
//        sleep(5000);
    }
    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        /*
         * The underlying datalogger object - it cares only about an array of loggable fields
         */
        private final Datalogger datalogger;
        /*
         * These are all of the fields that we want in the datalog.
         * Note: Order here is NOT important. The order is important
         *       in the setFields() call below
         */
        public Datalogger.GenericField loopCounter = new Datalogger.GenericField("LoopCounter");
        public Datalogger.GenericField runTime = new Datalogger.GenericField("RunTime");
        public Datalogger.GenericField power = new Datalogger.GenericField("Power");
        public Datalogger.GenericField bearing = new Datalogger.GenericField("Bearing");
        public Datalogger.GenericField error = new Datalogger.GenericField("Error");
        public Datalogger.GenericField tagCounter = new Datalogger.GenericField("Tag Cnt");
        public Datalogger.GenericField inLoopCounter = new Datalogger.GenericField("Run In Loop");

        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    /*
                     * Tell it about the fields we care to log.
                     * Note: Order *IS* important here! The order in which we list the
                     *       fields is the order in which they will appear in the log.
                     */
                    .setFields(
                            bearing,
                            power,
                            loopCounter,
                            inLoopCounter,
                            tagCounter,
                            runTime,
                            error
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}
