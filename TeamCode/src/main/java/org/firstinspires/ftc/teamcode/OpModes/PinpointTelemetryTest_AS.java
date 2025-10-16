package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Import your local GoBildaPinpointDriver
// Make sure GoBildaPinpointDriver.java is in your teamcode folder

@TeleOp(name="Pinpoint Telemetry Test for Android Studio", group="Android")
public class PinpointTelemetryTest_AS extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private ElapsedTime runtime = new ElapsedTime();

    // Variables to track changes
    private double lastX = 0.0;
    private double lastY = 0.0;
    private double lastHeading = 0.0;

    // Starting position storage
    private boolean firstRead = true;
    private double startX = 0.0;
    private double startY = 0.0;
    private double startHeading = 0.0;

    @Override
    public void init() {
        // Debug code to check what device is found
        try {
            HardwareDevice device = hardwareMap.get("odo");
            telemetry.addData("Device found", device.getClass().getSimpleName());
            telemetry.update();
            //sleep(3000); // Give time to read the message
        } catch (Exception e) {
            telemetry.addData("Debug Error", e.getMessage());
            telemetry.update();
            //sleep(3000);
        }

        // Initialize the Pinpoint sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure the Pinpoint sensor
        // IMPORTANT: Adjust these values for your robot!
        // These are example values - you'll need to measure your actual offsets
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.INCH); // X and Y offsets in mm

        // Set encoder resolution - adjust based on your odometry pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions - adjust if movement is inverted
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setYawScalar(-1.0); // Reverse the heading direction
        // Reset position and IMU
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized - Push robot around to test!");
        telemetry.addData("Instructions", "Watch X, Y, and Heading values");
        telemetry.addData("Note", "Values reset each time you run this program");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Keep updating during init
        pinpoint.update();

        Pose2D pos = pinpoint.getPosition();
        telemetry.addData("Status", "Waiting for start...");
        telemetry.addData("Current X (inches)", "%.2f", pos.getX(DistanceUnit.INCH));
        telemetry.addData("Current Y (inches)", "%.2f", pos.getY(DistanceUnit.INCH));
        telemetry.addData("Current Heading (degrees)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        // Reset the position one more time when starting
        pinpoint.resetPosAndIMU();
        firstRead = true;
    }

    @Override
    public void loop() {
        // Update the Pinpoint sensor
        pinpoint.update();

        // Get current position
        Pose2D pos = pinpoint.getPosition();
        double currentX = pos.getX(DistanceUnit.INCH);
        double currentY = pos.getY(DistanceUnit.INCH);
        double currentHeading = pos.getHeading(AngleUnit.DEGREES);

        // Store starting position on first read
        if (firstRead) {
            startX = currentX;
            startY = currentY;
            startHeading = currentHeading;
            lastX = currentX;
            lastY = currentY;
            lastHeading = currentHeading;
            firstRead = false;
        }

        // Calculate changes from last reading
        double deltaX = currentX - lastX;
        double deltaY = currentY - lastY;
        double deltaHeading = currentHeading - lastHeading;

        // Calculate total distance from start
        double totalDistance = Math.sqrt(Math.pow(currentX - startX, 2) + Math.pow(currentY - startY, 2));

        // Calculate movement speed (distance per second)
        double movementSpeed = 0.0;
        if (runtime.seconds() > 0) {
            movementSpeed = Math.sqrt(deltaX * deltaX + deltaY * deltaY) / 0.02; // Assuming ~50Hz loop
        }

        // === MAIN TELEMETRY ===
        telemetry.addData("=== CURRENT POSITION ===", "");
        telemetry.addData("X Position (inches)", "%.3f", currentX);
        telemetry.addData("Y Position (inches)", "%.3f", currentY);
        telemetry.addData("Heading (degrees)", "%.2f", currentHeading);
        telemetry.addData("", "");

        telemetry.addData("=== MOVEMENT FROM START ===", "");
        telemetry.addData("Distance from Start", "%.3f inches", totalDistance);
        telemetry.addData("X Change from Start", "%.3f inches", currentX - startX);
        telemetry.addData("Y Change from Start", "%.3f inches", currentY - startY);
        telemetry.addData("Heading Change from Start", "%.2f degrees", currentHeading - startHeading);
        telemetry.addData("", "");

        telemetry.addData("=== RECENT MOVEMENT ===", "");
        telemetry.addData("Recent X Change", "%.4f inches", deltaX);
        telemetry.addData("Recent Y Change", "%.4f inches", deltaY);
        telemetry.addData("Recent Heading Change", "%.3f degrees", deltaHeading);
        telemetry.addData("Movement Speed", "%.2f in/sec", movementSpeed);
        telemetry.addData("", "");

        telemetry.addData("=== SENSOR INFO ===", "");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Loop Frequency", "~%.0f Hz", 1.0 / runtime.seconds() * getRuntime());

        // Raw encoder values (if available)
        telemetry.addData("=== RAW DATA ===", "");
        telemetry.addData("X Encoder", pinpoint.getEncoderX());
        telemetry.addData("Y Encoder", pinpoint.getEncoderY());
        telemetry.addData("IMU Heading (deg)", "%.2f", pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("", "");
        telemetry.addData("INSTRUCTIONS", "Push robot around manually");
        telemetry.addData("WATCH FOR", "Smooth, accurate position tracking");

        telemetry.update();

        // Store current values for next loop
        lastX = currentX;
        lastY = lastY;
        lastHeading = currentHeading;
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Test Complete");
        telemetry.addData("Final Position", "X: %.2f, Y: %.2f, H: %.1f°",
                lastX, lastY, lastHeading);
        telemetry.update();
    }
}