/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTags_dco;
import org.firstinspires.ftc.teamcode.BuildConfig;

@TeleOp(name="MecanumTeleop_dco", group="DCO")
public class MecanumTeleop_dco extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftDrive,  backLeftDrive, frontRightDrive, backRightDrive;

    private Servo directionServo;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    IMU imu;
    AprilTags_dco aprilTags;

    boolean slowTurn = false, reportCurrentAprilTagLoop;
    double currentBearing;
    double yawImu, drive, strafe, turn;
    double power = 0.7;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

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

        directionServo = hardwareMap.get(Servo.class, "direction");

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);

        aprilTags = new AprilTags_dco();
        aprilTags.initAprilTag(hardwareMap, telemetry, imu);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        aprilTags.scanForObelisk();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        aprilTags.runInLoop(telemetry, false);

        currentBearing = aprilTags.getBearing();
        yawImu = imu.getRobotYawPitchRollAngles().getYaw();

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

//        if (gamepad1.x) {
//            reportCurrentAprilTagLoop = false;
//            imu.resetYaw();
//            rotateTo(aprilTags.getBearing());
//            reportCurrentAprilTagLoop = true;
//        }

        drive = -power * gamepad1.left_stick_y; // forward/back
        strafe = gamepad1.left_stick_x; // left/right
        if (slowTurn) {
            turn = 0.1 * gamepad1.right_stick_x;  // rotation
        } else {
            turn = gamepad1.right_stick_x;  // rotation
        }

        drive(drive, strafe, turn);

        telemetry.addData("LED:", aprilTags.getLedColor());
        telemetry.addData("IMU Yaw:", yawImu);
        telemetry.addData("Bearing:", currentBearing);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

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
}
