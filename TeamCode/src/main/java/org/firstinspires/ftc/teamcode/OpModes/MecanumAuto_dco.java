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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTags_dco;
import org.firstinspires.ftc.teamcode.Datalogger;

@Autonomous(name="DCO: Mecanum Auto")
public class MecanumAuto_dco extends LinearOpMode {

    public static boolean logData = true;
    private final int NEVERREST_TICKS_PER_REV = 1120;
    private final double DIAMETER_GREY = 3.5;
    private final double DIAMETER_GOBILDA = 100./10/2.54;
    private int distance = 0;
    private double diameter = 0;
    private String wheels = "";
    public static int decimation = 1;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    int eee;

    private String allianceSide = "";
    private int allianceId = 0;

    DcMotor frontLeftDrive,  backLeftDrive, frontRightDrive, backRightDrive;
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

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        boolean inputComplete = false;
//        while (!inputComplete) {
//            telemetry.addLine("Select LB: to use GoBilda wheels");
//            telemetry.addLine("             RB: to use grey mecanum wheels\n");
//            telemetry.addLine("Select A: to decrease distance");
//            telemetry.addLine("             Y: to increase distance\n");
//            telemetry.addLine(String.format("Wheels: %s", wheels));
//            telemetry.addLine(String.format("Alliance side: %d (%s)", allianceId, allianceSide));
//            telemetry.addLine(String.format("Distance: %d: (in)", distance));
//            telemetry.update();
//            /*
//             * Code that runs REPEATEDLY after the driver hits INIT, but before you hit START
//             *    Tasks to include are,
//             *       - Allow driver to set the alliance color
//             */
//
//            if (gamepad1.right_bumper) {
//                wheels = "Grey";
//                diameter = DIAMETER_GREY;
//            } else if (gamepad1.left_bumper) {
//                wheels = "GoBilda";
//                diameter = DIAMETER_GOBILDA;
//            }
//
//            if (gamepad1.yWasPressed()) {
//                distance = distance + 1;
//            } else if (gamepad1.aWasPressed()) {
//                distance = distance - 1;
//                distance = Math.max(distance,0);
//            }
//        }
//        int ticks = CalculateDistanceInTicks(distance,diameter/2);
//        telemetry.addLine(String.format("Radius: %4.2f", diameter/2));
//        telemetry.addLine(String.format("Distance: %d", distance));
//        telemetry.addLine(String.format("Ticks: %d (%d)",ticks,eee));
//        telemetry.addLine("Waiting to start match.");
//        telemetry.update();
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
        aprilTags.initAprilTag(hardwareMap, telemetry, decimation);

        do {
            if (gamepad1.x) {
                frontLeftDrive.setPower(0.5);
            } else {
                frontLeftDrive.setPower(0.);
            }
            if (gamepad1.y) {
                frontRightDrive.setPower(0.5);
            } else {
                frontRightDrive.setPower(0.);
            }
            if (gamepad1.a) {
                backLeftDrive.setPower(0.5);
            } else {
                backLeftDrive.setPower(0.);
            }
            if (gamepad1.b) {
                backRightDrive.setPower(0.5);
            } else {
                backRightDrive.setPower(0.);
            }
        } while (true);
//        do  {
//            aprilTags.scanForObelisk();
//        } while(opModeInInit());
        //waitForStart();
        //if (isStopRequested()) return;

//        DriveForwardDistance(0.75, distance);
//        DriveForwardForTime(0.75, 1.0);
//
//        rotateRobot();
    }

    @SuppressLint("DefaultLocale")
    public void rotateRobot() {
        double Kp                = 0.03;   // start small, increase until responsive
        double minPower          = 0.12;    // overcome stiction
        double maxPower          = 0.6;     // safety cap
        double tolerance         = 1.0;     // degrees
        double target            = 0.;
        double bearing;
        int i = 0;
        double rotationDirection = 0;       // CW = 1, CCW = -1

        int tagId = aprilTags.getTagId();

        aprilTags.getCurrentPosition(tagId);
        bearing = aprilTags.getBearing();

        telemetry.addLine(String.format("Bearing: %6.2f", bearing));
        telemetry.addLine(String.format("ID: %d", tagId));
        telemetry.update();

        while (true) {

            aprilTags.getCurrentPosition(tagId);
            bearing = aprilTags.getBearing();
            double error = target - bearing;

            if (Math.abs(error) <= tolerance) break;

            double power = Kp * error;

            /* Data log
             * Note: The order in which we set datalog fields does *not* matter!
             *       Order is configured inside the Datalog class constructor.
             */
            if (logData) {
                datalog.loopCounter.set(i);
                datalog.tagCounter.set(aprilTags.getCurrentPositionCounter());
                datalog.inLoopCounter.set(aprilTags.getRunInLoopCounter());
                datalog.runTime.set(runtime.seconds());
                datalog.bearing.set(bearing);
                datalog.power.set(power);
                datalog.error.set(error);
                datalog.writeLine();
            }
            i++;

            telemetry.addLine(String.format("Counter: %d", i++));
            telemetry.addLine(String.format("Bearing: %6.2f", bearing));
            telemetry.addLine(String.format("Power: %6.2f", power));
            telemetry.addLine(String.format("Error: %6.2f", error));
            telemetry.update();

//            if (tagId==20) {             // blue
//                rotationDirection = -1.;
//            } else if (tagId==24) {      // red
//                rotationDirection = 1.;
//            }

            // enforce min power so robot actually moves
            if (Math.abs(power) < minPower) power = Math.signum(power) * minPower;
//            if (Math.abs(power) < minPower) power = rotationDirection * minPower;
            // clamp to max
            power = Math.max(-maxPower, Math.min(maxPower, power));

            frontLeftDrive.setPower(power);
            frontRightDrive.setPower(-power);
            backLeftDrive.setPower(power);
            backRightDrive.setPower(-power);
            sleep(50);

//            if (aprilTags.getBearing() > 2. || aprilTags.getBearing() < -2.) {
//                frontLeftDrive.setPower(rotationDirection * power);
//                frontRightDrive.setPower(rotationDirection * power);
//                backLeftDrive.setPower(rotationDirection * power);
//                backRightDrive.setPower(rotationDirection * power);
//
//                telemetry.addLine(String.format("Bearing: %6.2f", aprilTags.getBearing()));
//                telemetry.update();
//            } else {
//                break;
//            }
        }
        StopDriving();
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
