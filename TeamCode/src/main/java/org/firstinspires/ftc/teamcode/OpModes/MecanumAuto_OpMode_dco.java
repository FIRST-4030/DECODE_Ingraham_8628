/* Copyright (c) 2025 FIRST. All rights reserved.
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

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.AprilTags_dco;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.Datalogger;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "DCO: Mecanum Auto - OpMode")
@Disabled
public class MecanumAuto_OpMode_dco extends OpMode {

    public static boolean logData = true;
    public static int decimation = 3;
    public static double power = 0.7;

    boolean slowTurn = false;

    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;
    private int i = 0; // loop counter

    Servo directionServo;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    AprilTags_dco aprilTags;
    int side;
    Datalog datalog;
    ElapsedTime runtime = new ElapsedTime();
    YawPitchRollAngles orientation;
    double yawImu;
    double drive, strafe, turn;
    boolean reportCurrentAprilTagLoop;

    @SuppressLint("DefaultLocale")
    @Override
    public void init() {

        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        directionServo = hardwareMap.get(Servo.class, "direction");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
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

        aprilTags = new AprilTags_dco();
        aprilTags.initAprilTag(hardwareMap, telemetry, decimation);

        // Initialize the datalog
        if (logData) {
            datalog = new Datalog(String.format("AprilTagLog_%d_%4.2f", decimation, power));
        }

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        reportCurrentAprilTagLoop = true;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop() {
        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);

        if (gamepad1.x) {
            side = 20;
        } else if (gamepad1.b) {
            side = 24;
        }

        telemetry.addData("side:", side);
        telemetry.addLine(String.format("AprilTagLog_%d_%4.2f", decimation, power));

        telemetry.update();
    }

    @Override
    public void loop() {

        aprilTags.runInLoop(telemetry);

        String ledColor = aprilTags.getLedColor();
        if (ledColor.equals("green")) {
            redLED.setState(false);
            greenLED.setState(true);
            directionServo.setPosition(0.4);
        } else {
            redLED.setState(true);
            greenLED.setState(false);
            directionServo.setPosition(0.75);
        }

        if (gamepad1.a) {
            imu.resetYaw();
        }

        if (gamepad1.x) {
            reportCurrentAprilTagLoop = false;
            zeroRobot();
        }

        orientation = imu.getRobotYawPitchRollAngles();
        yawImu = orientation.getYaw();

        drive = -power * gamepad1.left_stick_y; // forward/back
        strafe = gamepad1.left_stick_x; // left/right
        if (slowTurn) {
            turn = 0.1 * gamepad1.right_stick_x;  // rotation
        } else {
            turn = gamepad1.right_stick_x;  // rotation
        }

        drive(drive, strafe, turn);

        /* Data log
         * Note: The order in which we set datalog fields does *not* matter!
         *       Order is configured inside the Datalog class constructor.
         */
        if (logData) {
            datalog.loopCounter.set(i);
            datalog.runTime.set(runtime.seconds());
            datalog.ledColor.set(ledColor);
            datalog.yawApril.set(aprilTags.getYaw());
            datalog.yawImu.set(yawImu);
            datalog.bearing.set(aprilTags.getBearing());
            datalog.range.set(aprilTags.getRange());
            datalog.turn.set(turn);
            datalog.writeLine();
        }
        i++;
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 0.5;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, abs(frontLeftPower));
        maxPower = Math.max(maxPower, abs(frontRightPower));
        maxPower = Math.max(maxPower, abs(backRightPower));
        maxPower = Math.max(maxPower, abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    @Override
    public void stop() {
        aprilTags.closeAprilTag();
        redLED.setState(false);
        greenLED.setState(false);
    }

    private void zeroRobot() {
        double currentBearing = aprilTags.getBearing();
        double kP = 0.02;

        while (Math.abs(currentBearing) > 2) {
            double turnPower = Range.clip(kP * currentBearing, -0.4, 0.4);

            // Rotate robot (positive = turn right)
            frontLeftDrive.setPower(turnPower);
            backLeftDrive.setPower(turnPower);
            frontRightDrive.setPower(-turnPower);
            backRightDrive.setPower(-turnPower);

            telemetry.addData("Tag ID", aprilTags.getTagId());
            telemetry.addData("Bearing (deg)", "%.2f", currentBearing);
            telemetry.addData("Turn Power", "%.2f", turnPower);
            telemetry.update();
            currentBearing = aprilTags.getBearing();
        }

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        sleep(100);
        telemetry.addLine("Aligned with AprilTag");
        reportCurrentAprilTagLoop = true;
        telemetry.update();
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
        public Datalogger.GenericField ledColor = new Datalogger.GenericField("ledColor");
        public Datalogger.GenericField yawApril = new Datalogger.GenericField("yawApril");
        public Datalogger.GenericField yawImu = new Datalogger.GenericField("yawIMU");
        public Datalogger.GenericField bearing = new Datalogger.GenericField("bearing");
        public Datalogger.GenericField range = new Datalogger.GenericField("range");
        public Datalogger.GenericField turn = new Datalogger.GenericField("turn");

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
                            yawApril,
                            yawImu,
                            loopCounter,
                            runTime,
                            ledColor,
                            bearing,
                            range,
                            turn
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
