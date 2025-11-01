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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.ShooterVelo;
//import org.firstinspires.ftc.teamcode.Shooter;

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
@TeleOp(name = "MecanumTeleop", group = "Robot")
public class MecanumTeleop extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotorEx collector;
    ShooterVelo shooter;
    Servo shooterHinge;
    //    HelperAprilTag_Nf helperAprilTag;
    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    double shooterSpeedIncrement = 2.0;  // RPS rotations per second
    double currentVelocity = 20.0;  // RPS  rotations per second
    boolean shooting = false; // true when shooting sequence begins
    double collectorSpeed=0.4;

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

        shooter=new ShooterVelo(hardwareMap,"shooter",false);

//        helperAprilTag = new HelperAprilTag_Nf();
//        helperAprilTag.initAprilTag(hardwareMap);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collector = hardwareMap.get(DcMotorEx.class, "collector");

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collector.setDirection(DcMotor.Direction.FORWARD);

        shooterHinge = hardwareMap.get(Servo.class, "shooterHinge");
        shooterHinge.setPosition(0.7);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    private double driveSlower = 1;
    @Override
    public void loop() {

//        helperAprilTag.telemetryAprilTag(telemetry);

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing

        //Gamepad 1
        if (gamepad1.start) {
            imu.resetYaw();
        }

        if (gamepad1.leftBumperWasPressed()) {
            driveSlower=0.3;
        }
        if (gamepad1.leftBumperWasReleased()) {
            driveSlower=0.75;
        }


        //Gamepad 2
        if (gamepad2.start) {
            imu.resetYaw();
        }

        if (gamepad2.leftBumperWasPressed()) {
            shooting = true;
        }
        if (shooting) {
            shooter.setTargetVelocity(shooterSpeedIncrement);
            shooter.overridePower();
            if (shooter.atSpeed()) {
                shooterHinge.setPosition(0.0);
                sleep(500);
                shooterHinge.setPosition(0.7);
                shooting = false;
            }
            shooter.setTargetVelocity(0);
        }

        if (gamepad2.dpadUpWasPressed()) {
            currentVelocity += shooterSpeedIncrement;
            shooter.setTargetVelocity(shooterSpeedIncrement);
        }

        if (gamepad2.dpadDownWasPressed()) {
            currentVelocity -= shooterSpeedIncrement;
            shooter.setTargetVelocity(-shooterSpeedIncrement);
        }

        if (gamepad2.bWasPressed()) {
            collector.setPower(collectorSpeed);
        }
        if (gamepad2.aWasPressed()) {
            collector.setPower(0.0);
        }

        telemetry.addData("collector current velocity:", collector.getVelocity());
        telemetry.addData("collector target power", collectorSpeed);
        telemetry.addData("Shooter Current Velocity:", shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity: ", currentVelocity);

        telemetry.update();


//        //Working out the boolean methods for the triggers. I'll leave this code commented out. -Elijah
//        if (gamepad1.left_trigger >= 0.2) {
//            shooter.setPower( 0.7 * (gamepad1.left_trigger));
//            //replace with shooter.setPower(1); for set power instead of variable via float.
//        }
//        else {
//            shooter.setPower(0);
//        }
//
//        telemetry.addLine("shooterPower: "+gamepad1.left_trigger);
//        telemetry.update();
//
//        //This code works, if you want to implement it. It changes the shooter power based on
//        //float gamepad1.left_trigger, multiplied by a limiter.
//
//        //End of my tests and edits.

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
          // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
