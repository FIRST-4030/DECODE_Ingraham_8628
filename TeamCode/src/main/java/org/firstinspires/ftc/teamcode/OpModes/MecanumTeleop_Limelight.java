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

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.Shooter;

@Configurable
@TeleOp(name = "MecanumTeleop Limelight", group = "Robot")
public class MecanumTeleop_Limelight extends OpMode {

    public static int volley_delay = 200;

    Chassis chassis;
    DcMotorEx collector;
    Shooter shooter;
    Servo liftServo;
    Limelight limelight;

    IMU imu;
    ElapsedTime shotTimer = new ElapsedTime();

    boolean shoot1 = false, shoot3 = false; // true when shooting sequence begins
    double collectorSpeed = 0.5;
    boolean shooterOn = false;
    boolean targetInView;
    boolean collectorOn = false;

    @Override
    public void init() {

        chassis = new Chassis(hardwareMap);

        shooter=new Shooter(hardwareMap,"shooter",true);

        collector = hardwareMap.get(DcMotorEx.class, "collector");

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collector.setDirection(DcMotor.Direction.REVERSE);

        liftServo = hardwareMap.get(Servo.class, "liftServo");
        liftServo.setPosition(1.0);  // Up with Axon Max is 1.0,  was 0.0 with Savox

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight = new Limelight();
        limelight.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        limelight.readObelisk();

        if (gamepad1.xWasPressed() && gamepad1.right_bumper) {
            Blackboard.alliance = Blackboard.Alliance.BLUE;
            limelight.setTeam(20);
        } else if (gamepad1.bWasPressed() && gamepad1.right_bumper) {
            Blackboard.alliance = Blackboard.Alliance.RED;
            limelight.setTeam(24);
        }

        telemetry.addData("Pad 1, Left Bumper", "Slow Drive");
        telemetry.addData("Pad 1, Right Bumper", "Very Slow Drive");
        telemetry.addData("Pad 1, A", "Raise Robot");
        telemetry.addData("Pad 1, Y", "Lower Robot");
        telemetry.addData("--", "--");
        telemetry.addData("Pad 2, Left Bumper", "Shoot");
        telemetry.addData("Pad 2, B", "Collector On/Off");
        telemetry.addData("Pad 2, X", "Collector Reverse");

        telemetry.addData("Alliance", Blackboard.getAllianceAsString());
        telemetry.addLine("HOLD RB AND Press X to override alliance to BLUE");
        telemetry.addLine("HOLD RB AND Press B to override alliance to RED");

        telemetry.addData("Obelisk", limelight.getObelisk());
        telemetry.addData("Goal Tag ID", limelight.getTeam());

        telemetry.update();
    }

    public void start() {
        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            limelight.setTeam(24);
        }
        else if (Blackboard.alliance == Blackboard.Alliance.BLUE) {
            limelight.setTeam(20);
        }
    }

    @Override
    public void loop() {
        targetInView = limelight.process(telemetry);
        shooter.overridePower();

        telemetry.addData("Target is in view:", targetInView);
        telemetry.addData("Shooter Current Velocity", shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity", shooter.targetVelocity);
        telemetry.addData("Distance to Target", limelight.getRange());

        //Gamepad 1
        if (gamepad1.start) {
            imu.resetYaw();
        }

        //Slow Drive
        if (gamepad1.leftBumperWasPressed()) {
            chassis.setMaxSpeed(0.5);
        }
        if (gamepad1.leftBumperWasReleased()) {
            chassis.setMaxSpeed(1.0);
        }
        //Precision Drive
        if (gamepad1.rightBumperWasPressed()) {
            chassis.setMaxSpeed(0.2);
        }
        if (gamepad1.rightBumperWasReleased()) {
            chassis.setMaxSpeed(1.0);
        }

        if (gamepad1.yWasPressed()) {
            shooter.targetVelocity = 0;
            shooterOn = false;
            collector.setPower(0.0);
            collectorOn = false;
            liftServo.setPosition(1.0);
        }
        if (gamepad1.aWasPressed()) {
            shooter.targetVelocity = 0;
            shooterOn = false;
            collector.setPower(0.0);
            collectorOn = false;
            liftServo.setPosition(0.0);
        }

        //Gamepad 2
        if (gamepad2.start) {
            imu.resetYaw();
        }

        //Collector Controls
        if (gamepad2.bWasReleased()) {
            if (!collectorOn) {
                collector.setPower(collectorSpeed);
                collectorOn = true;
            } else {
                collector.setPower(0.0);
                collectorOn = false;
            }
        }

        if (gamepad2.xWasPressed()) {
            collector.setPower(-collectorSpeed);
        }
        if (gamepad2.xWasReleased()) {
            collector.setPower(0.0);
            collectorOn = false;
        }

        //Shooter Toggle
        if (gamepad2.leftBumperWasReleased()) {
            collector.setPower(0.0);
            collectorOn = false;
            shooter.setTargetVelocity(shooter.getShooterVelo(limelight));
            shoot1 = true;
            shotTimer.reset();
            shooterOn = false;
        }

        if (shoot1) {
            shotTimer.reset();
            shooter.shoot(shooter.getShooterVelo(limelight));
            shooter.stopShooter();
            shoot1 = false;
        }

        if (gamepad2.rightBumperWasReleased()) {
            collector.setPower(0.0);
            collectorOn = false;
            shooter.setTargetVelocity(shooter.getShooterVelo(limelight));
            shoot3 = true;
            shotTimer.reset();
            shooterOn = false;
        }

        if (shoot3) {
            shotTimer.reset();
            shooter.fireVolleySorted(limelight,telemetry,volley_delay);
            shooter.stopShooter();
            shoot3 = false;
        }

        telemetry.addData("Collector Current Power:", collector.getVelocity());
        telemetry.addData("Collector Target Power", collectorSpeed);
        telemetry.addData("Shooter Current Velocity:", shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity: ", shooter.targetVelocity);
        telemetry.update();

        chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}