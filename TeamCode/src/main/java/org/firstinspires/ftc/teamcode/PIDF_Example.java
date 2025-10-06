package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
 *  This OpMode uses the extended/enhanced PIDF-related functions of the DcMotorEx class.
 */

/**
 * This OpMode uses the extended/enhanced PIDF-related functions of the DcMotorEx class.
 * It also does Datalogging, to review the performance of the motor.
 */
@TeleOp(name="PIDF_example")
public class PIDF_Example extends LinearOpMode {

    // our DC motor
    DcMotorEx shooter;

    public static final double NEW_P = 150; // default is 10.0
    public static final double NEW_I = 0; // default is 3.0
    public static final double NEW_D = 0; // default is 0.0
    public static final double NEW_F = 15; // default is 0.0

    static final double   COUNTS_PER_REV = 28 ;  // REV HD Hex 1:1 Motor Encoder

    private double targetVelocity = 30;  // rotations per second (max is 60)
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    Datalog datalog; // create the data logger object

    private int i = 0; // loop counter

    // Timer
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        // Get reference to DC motor.
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get the LED colors from the hardwaremap
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        // Initialize the datalog
        datalog = new Datalog("ShooterLog");

        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        greenLED.setState(false);
        redLED.setState(true);

        // wait for start command
        waitForStart();

        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        PIDFCoefficients pidfOrig = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        PIDFCoefficients pidfModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Not sure if setVelocity is working properly
        // angular rate in counts (ticks) per second
        shooter.setVelocity(targetVelocity*COUNTS_PER_REV);

        // setPower is required, in addition to setVelocity
        shooter.setPower(targetVelocity/55); // max speed is about 55 RPS (empirically determined)

        runtime.reset(); // reset the clock

        // display info to user
        while(opModeIsActive()) {

            i++;

            double currentVelocity = shooter.getVelocity(AngleUnit.DEGREES)/COUNTS_PER_REV;

            // Change LEDs based on shooter velocity being in the "good" band
            if (currentVelocity > 0.9*targetVelocity && currentVelocity < 1.1*targetVelocity) {
                greenLED.setState(true);
                redLED.setState(false);
            } else {
                greenLED.setState(false);
                redLED.setState(true);
            }

            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.addData("shooterVelocity", currentVelocity);

            telemetry.update();

            // Data log
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            datalog.loopCounter.set(i);
            datalog.runTime.set(runtime.seconds());
            datalog.shooterVelocity.set(currentVelocity);
            datalog.targetVelocity.set(targetVelocity);
            datalog.writeLine();
        }
    }

/**
 * Datalog class encapsulates all the fields that will go into the datalog.
 */
public static class Datalog {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField loopCounter = new Datalogger.GenericField("LoopCounter");
    public Datalogger.GenericField runTime = new Datalogger.GenericField("RunTime");
    public Datalogger.GenericField deltaTime = new Datalogger.GenericField("deltaTime");
    public Datalogger.GenericField shooterVelocity = new Datalogger.GenericField("shooterVelocity");
    public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("targetVelocity");

    public Datalog(String name) {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        loopCounter,
                        runTime,
                        deltaTime,
                        shooterVelocity,
                        targetVelocity
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