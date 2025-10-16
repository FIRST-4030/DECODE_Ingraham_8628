package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="PinpointAutoSample for Android Studio", group="Android")
public class PinpointAutoSample_AS extends LinearOpMode {
    public static double TARGET_X = 15.0;
    public static double TARGET_Y = 0.0;
    public static double TARGET_HEADING = 0.0; // 90 degrees

    // Hardware declarations
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver odo;

    // Target positions on the field (in inches)

    // PID constants for movement
    private final double KP_TRANSLATION = 0.04;
    private final double KP_ROTATION = 0.5;

    // Movement thresholds
    private final double POSITION_TOLERANCE = 0.8;
    private final double HEADING_TOLERANCE = 4.0; // 4 degrees

    // Motor power limits
    private final double MAX_POWER = 0.5;
    private final double MIN_POWER = 0.12;

    // Safety limits
    private final double MAX_TRANSLATION_ERROR = 30.0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Wait for start
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Target", "X: %.1f, Y: %.1f, Heading: %.1f°",
                TARGET_X, TARGET_Y, TARGET_HEADING);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Reset odometry ONCE at the start
            telemetry.addData("Status", "Resetting odometry...");
            telemetry.update();

            odo.resetPosAndIMU();
            sleep(500);

            // Verify reset worked
            odo.update();
            Pose2D startPos = odo.getPosition();
            telemetry.clear();
            telemetry.addData("Status", "Ready to start movement");
            telemetry.addData("Start Position", "X: %.2f, Y: %.2f, H: %.2f°",
                    startPos.getX(DistanceUnit.INCH),
                    startPos.getY(DistanceUnit.INCH),
                    startPos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("", "Press A to START");
            telemetry.update();

            // Wait for A button
            while (opModeIsActive() && !gamepad1.a) {
                sleep(50);
            }

            runtime.reset();

            // Move to target position
            moveToPosition(TARGET_X, TARGET_Y, TARGET_HEADING);

            // Stop all motors
            stopAllMotors();

            telemetry.addData("Status", "Movement Complete!");
            telemetry.update();
            sleep(2000);
        }
    }

    private void initializeHardware() {
        // Initialize drive motors
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Set all motors to FORWARD
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to run without encoders
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Pinpoint odometry sensor - EXACT SAME AS TELEMETRY TEST
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        odo.setOffsets(-84.0, -168.0, DistanceUnit.INCH);
        odo.setOffsets(127.0, -114.0, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setYawScalar(-1.0);
        odo.resetPosAndIMU();
    }

    private void moveToPosition(double targetX, double targetY, double targetHeading) {
        ElapsedTime moveTimer = new ElapsedTime();
        moveTimer.reset();

        int loopCount = 0;

        while (opModeIsActive() && moveTimer.seconds() < 10.0) {
            loopCount++;

            // Update odometry
            odo.update();

            // Get current position
            Pose2D pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.INCH);
            double currentY = pos.getY(DistanceUnit.INCH);

            // Get heading - show both raw and wrapped values
            double rawHeading = pos.getHeading(AngleUnit.DEGREES);
            double currentHeading = angleWrapDegrees(rawHeading);

            // DEBUG TELEMETRY - Show every loop
            telemetry.clear();
            telemetry.addData("=== LOOP", loopCount);
            telemetry.addData(">>> RAW HEADING", "%.1f degrees", rawHeading);
            telemetry.addData(">>> WRAPPED HEADING", "%.1f degrees", currentHeading);
            telemetry.addData("Position X, Y", "%.1f, %.1f", currentX, currentY);
            telemetry.addData("Target Heading", "%.1f degrees", targetHeading);

            // Calculate errors
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double errorHeading = angleWrapDegrees(targetHeading - currentHeading);

            telemetry.addData("Heading Error", "%.1f degrees", errorHeading);

            // Safety check
            double positionError = Math.sqrt(errorX * errorX + errorY * errorY);
            if (positionError > MAX_TRANSLATION_ERROR) {
                telemetry.addData("ERROR", "Position error too large!");
                telemetry.update();
                break;
            }

            // Check if reached target
            if (Math.abs(errorX) < POSITION_TOLERANCE &&
                    Math.abs(errorY) < POSITION_TOLERANCE &&
                    Math.abs(errorHeading) < HEADING_TOLERANCE) {
                telemetry.addData(">>> SUCCESS", "Target reached!");
                telemetry.update();
                break;
            }

            // Calculate drive powers
            double driveX = errorX * KP_TRANSLATION;
            double driveY = errorY * KP_TRANSLATION;
            double rotate = errorHeading * KP_ROTATION;

            // Convert field-relative to robot-relative
            double headingRad = Math.toRadians(currentHeading);
            double robotX = driveX * Math.cos(-headingRad) - driveY * Math.sin(-headingRad);
            double robotY = driveX * Math.sin(-headingRad) + driveY * Math.cos(-headingRad);

            // Calculate wheel powers
            double frontLeftPower = robotY - robotX + rotate;
            double frontRightPower = robotY + robotX - rotate;
            double backLeftPower = robotY + robotX + rotate;
            double backRightPower = robotY - robotX - rotate;

            // Normalize wheel powers
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (maxPower > MAX_POWER) {
                frontLeftPower *= MAX_POWER / maxPower;
                frontRightPower *= MAX_POWER / maxPower;
                backLeftPower *= MAX_POWER / maxPower;
                backRightPower *= MAX_POWER / maxPower;
            }

            // Apply minimum power threshold
            if (maxPower > 0 && maxPower < MIN_POWER) {
                double scale = MIN_POWER / maxPower;
                frontLeftPower *= scale;
                frontRightPower *= scale;
                backLeftPower *= scale;
                backRightPower *= scale;
            }

            // Set motor powers
            leftFront.setPower(frontLeftPower);
            rightFront.setPower(frontRightPower);
            leftBack.setPower(backLeftPower);
            rightBack.setPower(backRightPower);

            telemetry.addData("Motor Powers", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();

            sleep(20);
        }
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private double angleWrapDegrees(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}