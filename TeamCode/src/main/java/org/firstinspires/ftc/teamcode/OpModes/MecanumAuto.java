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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTag_E;
import org.firstinspires.ftc.teamcode.ShooterVelo;

@Autonomous(name="Mecanum Auto", group="Linear OpMode")
public class MecanumAuto extends LinearOpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotorEx collector;
    ShooterVelo shooter;
    Servo shooterHinge;

    ElapsedTime runtime = new ElapsedTime();

    AprilTag_E aprilTags;
    
    ElapsedTime collectorTime = new ElapsedTime();

    double obBearing, obDist;
    double collectorSpeed = 0.4;
    boolean redSide, blueSide;

    IMU imu;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        shooter = new ShooterVelo(hardwareMap, "shooter", true);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collector = hardwareMap.get(DcMotorEx.class, "collector");
        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setDirection(DcMotor.Direction.REVERSE);

        shooterHinge = hardwareMap.get(Servo.class, "shooterHinge");
        shooterHinge.setPosition(0.25);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        aprilTags = new AprilTag_E();
        aprilTags.initAprilTag(hardwareMap);
        long delaySeconds=0;

        do {
            aprilTags.scanField(telemetry);
            obBearing = aprilTags.getObeliskBearing();
            obDist = aprilTags.getObeliskRange();

            telemetry.addData("Obelisk Bearing ", obBearing);
            telemetry.addData("Obelisk Range ", obDist);
            if (obBearing > 0) {
                telemetry.addData("SIDE ", "RED");
                redSide = true;
                blueSide = false;
            }
            if (obBearing < 0 && obBearing > -30) {
                telemetry.addData("SIDE ", "BLUE");
                redSide = false;
                blueSide = true;
            }
            telemetry.addData("press x to add 1 sec to delay",delaySeconds);
            telemetry.addData("press y to remove 1 sec from delay",delaySeconds);
            telemetry.addData("Range to Obelisk AprilTag", aprilTags.getObeliskRange());

            if (aprilTags.getObeliskRange() > 100) telemetry.addData("Field Position", "Far");
            if (aprilTags.getObeliskRange() < 100) telemetry.addData("Field Position", "Close");

            if (gamepad1.xWasPressed()) {
                delaySeconds ++;
            }
            if (gamepad1.yWasPressed()) {
                delaySeconds --;
            }
            telemetry.update();
        } while (opModeInInit());

        runtime.reset();
        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && obDist > 0) {

            sleep(delaySeconds * 1000);

            rotateTo(aprilTags.getBearing());

            shootShooter(35.0);
            shootShooter(35.0);
            shootShooter(35.0);
            stopShooter();
            moveForward(1.0, 400);

            //rotate(1130, 1);

            //collector.setPower(collectorSpeed);

            //moveForward(-0.25, 2650);

            //collectorTime.reset();
            //while (collectorTime.milliseconds() < 1000) collector.setPower(collectorSpeed);

            //collector.setPower(0);

            break;
        }
    }

    private void moveForward(double power, double mseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < mseconds) {
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            backRightDrive.setPower(power);
        }

        stopMotors();
    }

    private void stopMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void shootShooter(double velocity) {
        shooter.setTargetVelocity(velocity);
        ElapsedTime shooterTimer = new ElapsedTime();

        while (!shooter.atSpeed()) {
            shooter.overridePower();
        }

        shooterTimer.reset();
        shooterHinge.setPosition(0.55);

        while (shooterTimer.seconds() < 2) {
            shooter.overridePower();
        }

        shooterHinge.setPosition(0.25);

        while (shooterTimer.seconds() < 3) {
            shooter.overridePower();
        }
    }

    public void stopShooter() {
        shooter.setTargetVelocity(0);
        shooter.overridePower();
    }

    private void rotate (double milliseconds, int reverse) {
        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();
        int leftSidepos;

        if (blueSide) {
            leftSidepos = reverse;
        }
        else {
            leftSidepos = -reverse;
        }

        while (turnTimer.milliseconds() < milliseconds) {
            frontLeftDrive.setPower(leftSidepos * -0.5);
            backLeftDrive.setPower(leftSidepos * -0.5);
            frontRightDrive.setPower(leftSidepos * 0.5);
            backRightDrive.setPower(leftSidepos * 0.5);
        }

        stopMotors();
    }

    private void rotateTo(double targetAngle) {
        double Kp = 0.2;  // Proportional gain (tune this)
        double Kd = 0.005;  // derivative gain
        double minPower = 0.3;
        double maxPower = 0.5;
        double tolerance = 2.0;// degrees
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

            turnPower = Kp * error + Kd * derivative;

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
        stopMotors();
    }
}
