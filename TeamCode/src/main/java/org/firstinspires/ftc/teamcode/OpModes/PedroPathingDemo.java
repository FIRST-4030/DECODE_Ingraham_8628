package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
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

    // The order of values listed in Options is irrelevant
    enum Options { STOP, MOVE_LEFT, MOVE_FORWARD, FALL_BACK, MOVE_RIGHT }
    Options option;

    boolean doAutonomous = false;
    /*
     * Note: Pedro’s coordinate system spans an interval of [0, 144] on both the
     *       x and y axes, with (0, 0) defined as the bottom-left corner of the field.
     *
     * The following code is an attempt to show how to define a set of paths to
     * move the root in a square. As of the writing of this opMode I do not have
     * a tuned robot to see if it works.
     *
     * Start 20" from the left-hand corner of the field and against the audience wall
     */
    private final Pose startPose = new Pose(0, 20, Math.toRadians(0));
    /*
     * Move forward 20"
     */
    private final Pose forwardPose = new Pose(20, 20, Math.toRadians(0));
    /*
     * Move right 20" and rotate the robot so that it is facing right
     */
    private final Pose rightPose = new Pose(20, -90, Math.toRadians(-90));
    /*
     * Rotate the robot another 90 degrees and drive 15"
     */
    private final Pose backPose = new Pose(20, 5, Math.toRadians(-180));
    /*
     * Rotate the robot so that it is forward facing and return back to its start
     */
    private final Pose returnHomePose = new Pose(0, 20, Math.toRadians(0));

    PathChain moveLeft, moveForward, fallBack, moveRight, returnHome;

    @Override
    public void runOpMode() {

        chassis = new Chassis(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);   //set your starting pose

        buildPaths();

        runtime.reset();

        doAutonomous = true;
        option = Options.MOVE_FORWARD;   // Define the first action in the path

        while (opModeIsActive()) {
            follower.update();

            if (doAutonomous) {   //Step thru each path until they are exhausted
                autonomousPaths();
            }
        }
    }

    public void buildPaths() {

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

        returnHome = follower.pathBuilder()
                .addPath(new BezierLine(rightPose, returnHomePose))
                .setLinearHeadingInterpolation(rightPose.getHeading(), startPose.getHeading())
                .build();
    }

    void autonomousPaths() {
        switch (option) {
            case MOVE_LEFT:
                if (!follower.isBusy()) {
                    follower.followPath(moveLeft,true);
                }
                option = Options.STOP;
                break;
            case MOVE_FORWARD:
                if (!follower.isBusy()) {
                    follower.followPath(moveForward,true);
                    option = Options.MOVE_RIGHT;
                }
                break;
            case FALL_BACK:
                if (!follower.isBusy()) {
                    follower.followPath(fallBack,true);
                    option = Options.MOVE_LEFT;
                }
                break;
            case MOVE_RIGHT:
                if (!follower.isBusy()) {
                    follower.followPath(moveRight,true);
                    option = Options.FALL_BACK;
                }
                break;
            case STOP:
                if (!follower.isBusy()) {
                    doAutonomous = false;
                }
                break;
        }
    }
}
