package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.ControlHub;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsCompetition;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDemo;

/*
 * This opMode is provided to show how to introduce PedroPathing into a project.
 *
 * Before you build any paths you must tune your robot. Check out the following
 * link on how to go about proper tuning
 *     https://pedropathing.com/docs/pathing/tuning
 */
@Disabled
@Autonomous(name="PedroPathingDemoAuto")
public class PedroPathingDemoAuto extends LinearOpMode {

    public static double start_x = 56, start_y = 8, start_angle = 90;
    public static double inFrontOfBalls1_x = 40, inFrontOfBalls1_y = 35,inFrontOfBalls1_angle = 0;
    public static double behindBalls1_x = 13, behindBalls1_y = 35, behindBalls1_angle = 0;
    public static double moveToFreeSpace_x = 50, moveToFreeSpace_y = 35,moveToFreeSpace_angle = 0;
    public static double moveToFarShoot_x = 60, moveToFarShoot_y = 11,moveToFarShoot_angle = 110;

    public static ControlHub controlHub = new ControlHub();

    Chassis chassis;

    Constants constants;

    ElapsedTime runtime = new ElapsedTime();

    Follower follower;

    Datalog datalog;

    String compiledDate = BuildConfig.COMPILATION_DATE;
    PathChain InFrontOfBalls1, BehindBalls1, MoveToFreeSpace, MoveToFarShoot;

    boolean logData = false;

    // The order of values listed in Options is irrelevant
    enum Options { STOP, Do_InFrontOfBalls1, Do_BehindBalls1, Do_MoveToFreeSpace, Do_MoveToFarShoot }
    Options option;

    boolean doAutonomous = false;

    @Override
    public void runOpMode() {

        chassis = new Chassis(hardwareMap);
        Pose startPose = new Pose(start_x, start_y, Math.toRadians(start_angle));

        if (controlHub.getMacAddress().equals(Constants.PRIMARY_BOT)) {
            constants = new ConstantsCompetition();
        } else if (controlHub.getMacAddress().equals(Constants.SECONDARY_BOT)) {
            constants = new ConstantsDemo();
        } else {
            throw new RuntimeException("ControlHub MAC address did not match primary or secondary");
        }

        follower = constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);   //set your starting pose

        buildPaths();

        doAutonomous = true;
        option = Options.Do_InFrontOfBalls1;   // Define the first action in the path

        do {
            telemetry.addLine(String.format("Compiled on: %s",compiledDate));
            telemetry.addLine(String.format("Log Data (Y=Yes, A=No): %b",logData));
            telemetry.update();

            if (gamepad1.yWasReleased()) {
                logData = true;
            } else if (gamepad1.aWasReleased()) {
                logData = false;
            }
        } while (opModeInInit());

        // Initialize the datalog
        if (logData) {
            datalog = new Datalog("PedroPathingDemo");
        }

        runtime.reset();

        while (opModeIsActive()) {

            if (doAutonomous) {   //Step thru each path until they are exhausted
                autonomousPaths();
            }
        }
    }

    void buildPaths() {
        InFrontOfBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(start_x, start_y), new Pose(inFrontOfBalls1_x, inFrontOfBalls1_y)))
                .setLinearHeadingInterpolation(Math.toRadians(start_angle), Math.toRadians(inFrontOfBalls1_angle))
                .build();

        BehindBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(inFrontOfBalls1_x, inFrontOfBalls1_y), new Pose(behindBalls1_x, behindBalls1_y)))
                .setLinearHeadingInterpolation(Math.toRadians(inFrontOfBalls1_angle), Math.toRadians(behindBalls1_angle))
                .build();

        MoveToFreeSpace = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(behindBalls1_x, behindBalls1_y), new Pose(50., 35.)))
                .setLinearHeadingInterpolation(Math.toRadians(behindBalls1_angle), Math.toRadians(moveToFreeSpace_angle))
                .build();

        MoveToFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(moveToFreeSpace_x, moveToFreeSpace_y), new Pose(moveToFarShoot_x, moveToFarShoot_y)))
                .setLinearHeadingInterpolation(Math.toRadians(moveToFreeSpace_angle), Math.toRadians(moveToFarShoot_angle))
                .build();
    }

    void autonomousPaths() {
        follower.update();

        switch (option) {
            case Do_InFrontOfBalls1:
                if (!follower.isBusy()) {
                    follower.followPath(InFrontOfBalls1);
                    option = Options.Do_BehindBalls1;
                }
                break;
            case Do_BehindBalls1:
                if (!follower.isBusy()) {
                    follower.followPath(BehindBalls1);
                    option = Options.Do_MoveToFreeSpace;
                }
                break;
            case Do_MoveToFreeSpace:
                if (!follower.isBusy()) {
                    follower.followPath(MoveToFreeSpace);
                    option = Options.Do_MoveToFarShoot;
                }
                break;
            case Do_MoveToFarShoot:
                if (!follower.isBusy()) {
                    follower.followPath(MoveToFarShoot);
                    option = Options.STOP;
                }
                break;
            case STOP:
                if (!follower.isBusy()) {
                    doAutonomous = false;
                }
                break;
        }
        if (logData) { logOneSample(follower.getPose()); }
    }

    private void logOneSample(Pose pose) {
        datalog.runTime.set(runtime.seconds());
        datalog.xPose.set(pose.getX());
        datalog.yPose.set(pose.getY());
        datalog.heading.set(pose.getHeading());
        datalog.writeLine();
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
        public Datalogger.GenericField xPose   = new Datalogger.GenericField("X");
        public Datalogger.GenericField yPose   = new Datalogger.GenericField("Y");
        public Datalogger.GenericField heading = new Datalogger.GenericField("Heading");

        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    /*
                     * Tell it about the fields we care to log.
                     * Note: Order *IS* important here! The order in which we list the
                     *       fields is the order in which they will appear in the log.
                     */
                    .setFields( runTime, xPose, yPose, heading )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}
