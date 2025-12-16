package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "ShooterDataLogger")
public class ShooterVelocityDataLogger extends LinearOpMode{

    private Servo shooterHinge;
    GoalTagLimelight limelight;
    private double initPos = 0.5;
    private double kP = 0.14;

    private double goalRange;
    private double goalBearing;

    Chassis ch;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    Datalog aimTestDatalog; // create the data logger object
    private double targetPower = 0;

    private boolean goal;

    private int i = 0; // loop counter
    private int j = 0;

    private int k = 0;
    Shooter shooter;

    private boolean readyToShoot = false;
    public static final double NEW_P = 150.0; // default is 10.0
    public static final double NEW_I = 0; // default is 3.0
    public static final double NEW_D = 0; // default is 0.0
    public static final double NEW_F = 15.0; // default is 0.0


    @Override
    public void runOpMode() {

        //shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter = new Shooter(hardwareMap,"shooterLeft", true);
        shooter.setControllerValues(0.3,0.0243);
        ch = new Chassis(hardwareMap);

        shooterHinge = hardwareMap.get(Servo.class, "launchFlapLeft");
        //launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        limelight = new GoalTagLimelight();

        // Initialize the datalog
        aimTestDatalog = new Datalog("launch log");
        // wait for start command

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        limelight.init(hardwareMap,telemetry);

        do {
            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("team ID", limelight.getID());
            telemetry.update();
        } while(opModeInInit());

        // display info to user
        while (opModeIsActive()) {
            i++;
            k++;
            shooter.overridePower();

            limelight.process(telemetry);
            goalRange = limelight.getRange();
            goalBearing = limelight.getTx();
            double targetVelocity = shooter.targetVelocity;


            if (gamepad1.leftBumperWasPressed()) {
                goal = true;
                telemetry.addData("goal",goal);
                telemetry.addData("targetVelocity", targetVelocity);
                telemetry.update();
                aimTestDatalog.goalBool.set(goal);
                aimTestDatalog.targetPower.set(targetVelocity);
                aimTestDatalog.goalRange.set(goalRange);
                aimTestDatalog.goalBearing.set(goalBearing);
                aimTestDatalog.writeLine();
            } else if (gamepad1.rightBumperWasPressed()) {
                goal = false;
                telemetry.addData("goal", goal);
                telemetry.addData("targetVelocity", targetVelocity);
                telemetry.update();
                aimTestDatalog.goalBool.set(goal);
                aimTestDatalog.targetPower.set(targetVelocity);
                aimTestDatalog.goalRange.set(goalRange);
                aimTestDatalog.goalBearing.set(goalBearing);
                aimTestDatalog.writeLine();
            } else if (gamepad1.yWasPressed()) {
                shooter.targetVelocity += 0.005;
            } else if (gamepad1.aWasPressed()) {
                shooter.targetVelocity -= 0.005;
            } else if (gamepad1.right_trigger == 1) {
                shooterHinge.setPosition(0);
                i = 0;
                readyToShoot = false;
            }
            if (i > 500) {
               shooterHinge.setPosition(initPos);
               i = 0;
            }
            ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad1.a && limelight.isDataCurrent) {
                turnToAprilTagLimelight();
            }
//            if (readyToShoot) {
//                if (j < 1) {
//                    launchFlap.setPosition(0.3);
//                    k = 0;
//                    j++;
//                }
//                if (k > 200) {
//                    launchFlap.setPosition(0);
//                    j--;
//                }

            telemetry.addData("val", gamepad1.right_trigger);


            telemetry.addData("targetVelocity", targetVelocity );
            //telemetry.addData("currentPower", shooter.getPower());
            telemetry.addData("GoalRange", (goalRange));
            telemetry.addData("GoalBearing", (goalBearing));
            telemetry.update();

            // Data log
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            //datalog.targetVelocity.set(targetVelocity);
            //datalog.writeLine();
        }
    }

    public void turnToAprilTagLimelight() {
        turnTo(0,0);
    }
    private void turnTo(double variance, double setPoint) {
        double currentAngle = limelight.getTx();
        double error = setPoint - currentAngle;

        //if (Math.abs(error) > variance) {
        double power = kP*error;

        telemetry.addData("turn power", power);
        ch.moveAllMotors(-power,power,-power,power);
//            if (error > rightBound) { // rotate left
//                moveAllMotors(-power,power,-power,power);
//            } else if (error < leftBound) { // rotate right
//                moveAllMotors(power,-power,power,-power);
//            }
        //}
    }
    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField targetPower = new Datalogger.GenericField("targetVelocity");
        public Datalogger.GenericField goalBool = new Datalogger.GenericField("goalBool");
        public Datalogger.GenericField goalRange = new Datalogger.GenericField("goalRange");
        public Datalogger.GenericField goalBearing = new Datalogger.GenericField("goalBearing");

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
                            goalBool,
                            targetPower,
                            goalRange,
                            goalBearing
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


