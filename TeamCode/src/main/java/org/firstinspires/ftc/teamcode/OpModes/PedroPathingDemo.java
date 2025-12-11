package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * This opMode is provided to show how to introduce PedroPathing into a project.
 *
 * Before you build any paths you must tune your robot. Check out the following
 * link on how to go about proper tuning
 *     https://pedropathing.com/docs/pathing/tuning
 */
@Autonomous(name="PedroPathingDemo")
public class PedroPathingDemo extends LinearOpMode {

    Chassis chassis;

    ElapsedTime runtime = new ElapsedTime();

    Follower follower;

    Datalog datalog;

    String compiledDate = BuildConfig.COMPILATION_DATE;

    int heightX = 10, widthY = 30;
    boolean logData = false;

    // The order of values listed in Options is irrelevant
    enum Options { STOP, MOVE_LEFT, MOVE_FORWARD, MOVE_BACK, MOVE_RIGHT }
    Options option;

    boolean doAutonomous = false;
    /*
     * Note: Pedro’s coordinate system spans an interval of [0, 144] on both the
     *       x and y axes, with (0, 0) defined as the bottom-left corner of the field.
     *
     * The following code is an attempt to show how to define a set of paths to
     * move in a rectangle.
     *
     * Start at lower left-hand corner (0,0)
     */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    /*
     * Move to upper left-hand corner (heightX,0)
     */
    private final Pose forwardPose = new Pose(heightX, 0, Math.toRadians(0));
    /*
     * Move to upper right-hand corner (heightX, widthY) and rotate the robot so that it is facing right
     */
    private final Pose rightPose = new Pose(heightX, -widthY, Math.toRadians(0));
    /*
     * Move to lower right-hand corner (0, widthY) and rotate the robot so that is facing backwards
     */
    private final Pose backPose = new Pose(0, -widthY, Math.toRadians(0));
//    /*
//     * Move back to the origin (0,0) and rotate the robot so that is facing forward
//     */
//    private final Pose leftPose = new Pose(0, 0, Math.toRadians(0));

    PathChain moveLeft, moveForward, fallBack, moveRight;

    @Override
    public void runOpMode() {

        chassis = new Chassis(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);   //set your starting pose
        follower.setMaxPower(0.5);

        buildPaths();

        doAutonomous = true;
        option = Options.MOVE_FORWARD;   // Define the first action in the path

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

        moveForward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, forwardPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading())
                .build();

        fallBack = follower.pathBuilder()
                .addPath(new BezierLine(rightPose, backPose))
                .setLinearHeadingInterpolation(rightPose.getHeading(), backPose.getHeading())
                .build();

        moveRight = follower.pathBuilder()
                .addPath(new BezierLine(forwardPose, rightPose))
                .setLinearHeadingInterpolation(forwardPose.getHeading(), rightPose.getHeading())
                .build();

        moveLeft = follower.pathBuilder()
                .addPath(new BezierLine(rightPose, startPose))
                .setLinearHeadingInterpolation(rightPose.getHeading(), startPose.getHeading())
                .build();
    }

    void autonomousPaths() {
        follower.update();

        switch (option) {
            case MOVE_LEFT:
                if (!follower.isBusy()) {
                    follower.followPath(moveLeft,true);
                    option = Options.STOP;
                }
                break;
            case MOVE_FORWARD:
                if (!follower.isBusy()) {
                    follower.followPath(moveForward,true);
                    option = Options.MOVE_RIGHT;
                }
                break;
            case MOVE_BACK:
                if (!follower.isBusy()) {
                    follower.followPath(fallBack,true);
                    option = Options.MOVE_LEFT;
                }
                break;
            case MOVE_RIGHT:
                if (!follower.isBusy()) {
                    follower.followPath(moveRight,true);
                    option = Options.MOVE_BACK;
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
