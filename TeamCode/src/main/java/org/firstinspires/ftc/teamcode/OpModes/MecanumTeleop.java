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


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.AprilTag;
import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.ShooterVelo;

@TeleOp(name = "MecanumTeleop", group = "Robot")
public class MecanumTeleop extends OpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotorEx collector;
    ShooterVelo shooter;
    Servo shooterHinge;
    AprilTag aprilTags;

    IMU imu;
    ElapsedTime shotTimer = new ElapsedTime();

    double shooterSpeedIncrement = 2.0;  // RPS rotations per second
    double currentVelocity = 20.0;  // RPS  rotations per second
    boolean shooting = false; // true when shooting sequence begins
    double collectorSpeed = 0.4;
    boolean shooterOn = false;

    private double driveSlower = 1;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter=new ShooterVelo(hardwareMap,"shooter",true);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

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

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        aprilTags = new AprilTag();
        aprilTags.initAprilTag(hardwareMap);
        if ( Blackboard.alliance== Blackboard.Alliance.RED){
            aprilTags.SetgoalTagId(24);
        }else{
            aprilTags.SetgoalTagId(20);
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("PAD 1, LEFT BUMPER", "SLOW DOWN");
        telemetry.addData("--", "--");
        telemetry.addData("PAD 2, LEFT BUMPER", "SHOOT");
        telemetry.addData("PAD 2, UP", "FASTER SHOT");
        telemetry.addData("PAD 2, DOWN", "SLOWER SHOT");
        telemetry.addData("PAD 2, B", "COLLECT ON");
        telemetry.addData("PAD 2, A", "COLLECT OFF");
        telemetry.addData("PAD 2, X", "COLLECT REVERSE");

        telemetry.update();
    }

    @Override
    public void loop() {

        aprilTags.runInLoop(telemetry, false);

        //Gamepad 1
        if (gamepad1.start) {
            imu.resetYaw();
        }

        //Slow Drive
        if (gamepad1.leftBumperWasPressed()) {
            driveSlower = 0.3;
        }
        if (gamepad1.leftBumperWasReleased()) {
            driveSlower = 1;
        }

        //Precision Drive
        if (gamepad1.rightBumperWasPressed()) {
            driveSlower = 0.1;
        }
        if (gamepad1.rightBumperWasReleased()) {
            driveSlower = 1;
        }

        //Gamepad 2
        if (gamepad2.start) {
            imu.resetYaw();
        }

        //Shooter Controls
        if (shooterOn) {
            if (gamepad2.leftBumperWasPressed()) {
                shooting = true;
                shotTimer.reset();
            }
            shooter.overridePower();
            currentVelocity=(aprilTags.distanceToGoal + 202.17) / 8.92124;

            shooter.setTargetVelocity(currentVelocity);
            if (shooting) {
                shooter.setTargetVelocity(currentVelocity);
                if (shooter.atSpeed()) {
                    shooterHinge.setPosition(0.55);
                    if (shotTimer.milliseconds() > 500) {
                        shooterHinge.setPosition(0.25);
                        shooting = false;
                    }
                }
            }
        }
        else {
            shooter.overridePower();
            shooter.setTargetVelocity(0);
            }

        //Dpad Shooter Speed Control
        if (gamepad2.dpadUpWasPressed()) {
            currentVelocity += shooterSpeedIncrement;
            shooter.setTargetVelocity(currentVelocity);
        }

        if (gamepad2.dpadDownWasPressed()) {
            currentVelocity -= shooterSpeedIncrement;
            shooter.setTargetVelocity(currentVelocity);
        }

        if (gamepad2.dpadRightWasPressed()) {
            currentVelocity = 36;
            shooter.setTargetVelocity(currentVelocity);
        }

        if (gamepad2.dpadLeftWasPressed()) {
            currentVelocity = 30;
            shooter.setTargetVelocity(currentVelocity);
        }

        //Collector Controls
        if (gamepad2.bWasPressed()) {
            collector.setPower(collectorSpeed);
        }

        if (gamepad2.aWasPressed()) {
            collector.setPower(0.0);
        }

        if (gamepad2.xWasPressed()) {
            collector.setPower(-collectorSpeed);
        }

        //Shooter Toggle
        if (gamepad2.yWasReleased()) {
            if (shooterOn) {
                shooterOn = false;
            }
            else {
                shooterOn = true;
            }
        }

        telemetry.addData("collector current velocity:", collector.getVelocity());
        telemetry.addData("collector target power", collectorSpeed);
        telemetry.addData("Shooter Current Velocity:", shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity: ", currentVelocity);
        telemetry.addLine("dpad left: 30  |  dpad right: 36");

        telemetry.update();

        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,driveSlower);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate,double maxSpeed) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
          // make this slower for slower drive

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}