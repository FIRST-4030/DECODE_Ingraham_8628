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

import android.provider.ContactsContract;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTag;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous(name="Mecanum Auto", group="Linear OpMode")
public class MecanumAuto extends LinearOpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotorEx collector;
    Shooter shooter;
    Servo shooterHinge;

    ElapsedTime runtime = new ElapsedTime();

    AprilTag aprilTags;

    Servo liftServo;
    
    ElapsedTime collectorTime = new ElapsedTime();

    int sideInt;

    double obBearing, obDist;
    double collectorSpeed = 0.5;

    boolean redSide, blueSide;
    boolean smallFootprint = false;

    IMU imu;

    Datalog datalog = new Datalog("MecanumAutoLog");
    boolean logData = true;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        shooter = new Shooter(hardwareMap, "shooter", true);

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

        liftServo = hardwareMap.get(Servo.class, "liftServo");
        liftServo.setPosition(1.0);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        aprilTags = new AprilTag();
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
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("press a/b to toggle limited auto | Limited Auto", smallFootprint);


            if (gamepad1.xWasPressed()) {
                delaySeconds++;
            }
            if (gamepad1.yWasPressed()) {
                delaySeconds--;
            }

            if (gamepad1.aWasReleased() && !smallFootprint) {
                smallFootprint = true;
            }
            if (gamepad1.bWasReleased() && smallFootprint) {
                smallFootprint = false;
            }

            telemetry.update();
        } while (opModeInInit());

        runtime.reset();
        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (redSide) {
                sideInt = 1;
            }
            else {
                sideInt = -1;
            }

            sleep(delaySeconds * 1000);

            if (smallFootprint) {
                runSmallFootprint();
            }

            if (!smallFootprint) {
                if (obDist > 100) {
                    runFromFar();
                } else {
                    runFromClose();
                }
            }

            break;
        }
    }

    private void runFromFar() {
        if (redSide) {
            rotateTo(aprilTags.getBearing() - 2);
        }
        else {
            rotateTo (aprilTags.getBearing() - 3);
        }

        shootShooter();
        shootShooter();
        shootShooter();
        stopShooter();

        rotateTo(0);

        imu.resetYaw();

        moveForward(1.0, 400);

        rotateTo(90 * sideInt);

        imu.resetYaw();

        collector.setPower(collectorSpeed);

        moveForward(-0.25, 2700);

        collectorTime.reset();
        while (collectorTime.milliseconds() < 1000) collector.setPower(collectorSpeed);

        collector.setPower(0);

        moveForward(0.25, 2250);

        rotateTo(-115 * sideInt);

        moveForward(-0.25, 1500);

        if (redSide) { rotateTo(aprilTags.getBearing() - 7); }
        else { rotateTo(aprilTags.getBearing() + 2); }

        shootShooter();
        shootShooter();
        shootShooter();
        stopShooter();

        moveForward(0.5, 800);
    }

    private void runFromClose() {
        sideInt = -sideInt;
        rotateTo(-130 * sideInt);
        moveForward(-0.5, 1550);

        imu.resetYaw();

        shootShooter(); //29.0
        shootShooter();
        shootShooter();
        stopShooter();

        rotateTo(265 * sideInt);
        moveForward(0.5, 570);

        imu.resetYaw();

        rotateTo(-130 * sideInt);

        collector.setPower(collectorSpeed);

        moveForward(-0.2, 2850);

        collectorTime.reset();
        while (collectorTime.milliseconds() < 1000) collector.setPower(collectorSpeed);

        collector.setPower(0);

        imu.resetYaw();

//        moveForward(0.25, 2550);
//
//        rotateTo(39);
//
//        shootShooter();
//        shootShooter();
//        shootShooter();
//        stopShooter();
//
//        rotateTo(-220);
//
//        moveForward(0.5, 700);
    }

    private void runSmallFootprint() {
        double velocity = 34.0;

        if (redSide) { rotateTo(aprilTags.getBearing() - 2); }
        else { rotateTo(aprilTags.getBearing() - 3); }

        shootShooter();
        shootShooter();
        shootShooter();
        stopShooter();

        rotateTo(0);

        moveForward(0.5, 500);
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

    public void shootShooter() {
        shooter.targetVelocity = (aprilTags.distanceToGoal + 202.17) / 8.92124;
        ElapsedTime shooterTimer = new ElapsedTime();

        while (!shooter.atSpeed()) {
            shooter.overridePower();
        }

        shooterTimer.reset();
        shooterHinge.setPosition(0.55);

        while (shooterTimer.milliseconds() < 500) {
            shooter.overridePower();
        }

        shooterHinge.setPosition(0.25);

        while (shooterTimer.milliseconds() < 1000) {
            shooter.overridePower();
        }
    }

    public void stopShooter() {
        shooter.targetVelocity = 0;
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
            telemetry.addData("IMU Angle", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            frontLeftDrive.setPower(-turnPower);
            backLeftDrive.setPower(-turnPower);
            frontRightDrive.setPower(turnPower);
            backRightDrive.setPower(turnPower);

            if (logData) {
                datalog.runTime.set(runtime.seconds());
                datalog.bearing.set(aprilTags.getBearing());
                datalog.targetAngle.set(targetAngle);
                datalog.currentAngle.set(currentAngle);
                datalog.error.set(error);
                datalog.turnPower.set(turnPower);
                datalog.IMUAngle.set(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            }
        }
        stopMotors();
    }

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
        public Datalogger.GenericField runTime = new Datalogger.GenericField("runTime");
        public Datalogger.GenericField bearing = new Datalogger.GenericField("bearing");
        public Datalogger.GenericField currentAngle = new Datalogger.GenericField("currentAngle");
        public Datalogger.GenericField targetAngle = new Datalogger.GenericField("targetAngle");
        public Datalogger.GenericField error = new Datalogger.GenericField("error");
        public Datalogger.GenericField IMUAngle = new Datalogger.GenericField("IMUAngle");
        public Datalogger.GenericField turnPower = new Datalogger.GenericField("turnPower");


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
                        runTime, bearing, currentAngle, targetAngle, error, IMUAngle, turnPower
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
