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

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@TeleOp(name = "DCO: Mecanum and AprilTags")
public class MecanumAndAprilTags_dco extends OpMode {

//    public static int numBearings = 5;
    public static boolean logData = true;

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
    double turn, strafe;
    double turnPower = 0;
    double yawError = 0;
    int side;
    String ledColor;
    Datalog datalog;
    ElapsedTime runtime = new ElapsedTime();

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
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        aprilTags.initAprilTag(hardwareMap);

        // Initialize the datalog
        if (logData)  {datalog = new Datalog("AprilTagLog"); }

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);

        if(gamepad1.x){
            side = 20;
        }
        else if(gamepad1.b){
            side = 24;
        }

        telemetry.addData("side:", side);
        telemetry.update();
    }

    @Override
    public void start() {
        aprilTags.setSide(side);
        runtime.reset(); // reset the clock

//        aprilTags.visionPortal.resumeStreaming();
    }

    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y; // forward/back
        double strafe = gamepad1.left_stick_x; // left/right
        double turn = gamepad1.right_stick_x;  // rotation

        aprilTags.runInLoop(telemetry);

        telemetry.addData("Bearing:", "%4.2f", aprilTags.getBearing());
        telemetry.update();

        if (abs(aprilTags.getBearing()) > 15.0) {
            redLED.setState(true);
            greenLED.setState(false);
            directionServo.setPosition(0);
            ledColor = "red";
        } else {
            redLED.setState(false);
            greenLED.setState(true);
            directionServo.setPosition(0.5);
            ledColor = "green";
        }

        if (gamepad1.a) {
            imu.resetYaw();
        }

        drive(drive, strafe, turn);
//
//        if (gamepad1.x) {
//            if (!aprilTags.currentDetections.isEmpty()) {
//                AprilTagDetection tag = aprilTags.currentDetections.get(0);
//                double yawError = aprilTags.target.ftcPose.yaw; // Heading (degrees)
//
//                telemetry.addData("Tag ID", tag.id);
//                telemetry.addData("Yaw (Heading)", "%.2f", yawError);
//
////                double targetYaw = 0.0;
//                double kP = 0.02; // Tune this!
////                double error = tag.ftcPose.yaw - targetYaw;
//
//                turnPower = Range.clip(yawError * kP, -0.3, 0.3);
//                // Basic proportional control
////                forwardPower = -x * 0.05;
////                strafePower = -y * 0.05;
////                turnPower = -yaw * 0.02;
////
////                // Apply thresholds (dead zones)
////                if (Math.abs(x) < 1.0) forwardPower = 0;
////                if (Math.abs(y) < 1.0) strafePower = 0;
////                if (Math.abs(yaw) < 2.0) turnPower = 0;
//                telemetry.addData("Turn", "%.2f", turnPower);
//            } else {
//                telemetry.addLine("No tags visible");
//            }
//
//            telemetry.update();
//
//            if (Math.abs(yawError) > 2.0) {
//                frontLeftDrive.setPower(turnPower);
//                frontRightDrive.setPower(-turnPower);
//                backLeftDrive.setPower(turnPower);
//                backRightDrive.setPower(-turnPower);
//            }
//        }

        /* Data log
         * Note: The order in which we set datalog fields does *not* matter!
         *       Order is configured inside the Datalog class constructor.
         */
        if ((logData) && ((i % 10) == 0)) {  // slow down how many records are logged
            datalog.loopCounter.set(i);
            datalog.runTime.set(runtime.seconds());
            datalog.ledColor.set(ledColor);
            datalog.bearing.set(aprilTags.getBearing());
            datalog.range.set(aprilTags.getRange());
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
        public Datalogger.GenericField deltaTime = new Datalogger.GenericField("deltaTime");
        public Datalogger.GenericField ledColor = new Datalogger.GenericField("ledColor");
        public Datalogger.GenericField bearing = new Datalogger.GenericField("bearing");
        public Datalogger.GenericField range = new Datalogger.GenericField("range");

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
                    loopCounter,
                    runTime,
                    deltaTime,
                    ledColor,
                    bearing,
                    range
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
