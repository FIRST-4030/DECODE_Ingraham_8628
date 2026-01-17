package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.ControlHub;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.IterativeAutoStep;
import org.firstinspires.ftc.teamcode.IterativeAutoStepChain;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsCompetition;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDemo;

@Configurable
@Autonomous(name="Mecanum Auto Limelight", group="Linear OpMode")
public class MecanumAuto_Limelight extends LinearOpMode {

    // Pedro pathing constants (editable in panels)
    public static double farStartX = 56, farStartY = 8, farStartAngle = 90;
    public static double nearStartX = 24, nearStartY = 120, nearStartAngle = 315;

    public static double inFrontOfBalls1_x = 50, inFrontOfBalls1_y = 35,inFrontOfBalls1_angle = 0;
    public static double inFrontOfBalls2_x = 50, inFrontOfBalls2_y = 59 ,inFrontOfBalls2_angle = 0;
    public static double inFrontOfBalls3_x = 50, inFrontOfBalls3_y = 83 ,inFrontOfBalls3_angle = 0;
    public static double behindBalls1_x = 23, behindBalls1_y = 35, behindBalls1_angle = 0;
    public static double behindBalls2_x = 23, behindBalls2_y = 59, behindBalls2_angle = 0;
    public static double behindBalls3_x = 23, behindBalls3_y = 83, behindBalls3_angle = 0;

    public static long moveToInFrontOfBallsDelayMS = 250;
    public static long moveToBehindBallsDelayMS = 0;
    public static long moveToFarShootDelayMS = 0;
    public static long moveToNearShootDelayMS = 0;
    public static long shootThreeBallsDelayMS = 0;
    public static double collectorSpeed = 0.35;
    public static float collectingMaxPower = 0.6f;

    public static double moveToFreeSpace_x = 50, moveToFreeSpace_y = 35, moveToFreeSpace_angle = 0;
    public static double moveToFarShoot_x = 55, moveToFarShoot_y = 16, moveToFarShoot_angle = 111;
    public static double moveToNearShoot_x = 48, moveToNearShoot_y = 96, moveToNearShoot_angle = 135;

    Chassis chassis;
    Constants constants;
    DcMotorEx collector;
    Shooter shooter;
    Servo shooterHinge;
    IMU imu;

    ElapsedTime runtime = new ElapsedTime();

    Limelight limelight;

    Servo liftServo;

    ElapsedTime collectorTime = new ElapsedTime();

    double obeliskBearing, obeliskDistance;

    boolean limitedAutoEnabled = false;
    boolean nearAutoEnabled = false;

    Follower follower;
    PathChain inFrontOfBalls1, behindBalls1, inFrontOfBalls2, behindBalls2, inFrontOfBalls3, behindBalls3, moveToFreeSpace, moveToFarShoot, moveToNearShoot;
    IterativeAutoStepChain farAutoStepChain, nearAutoStepChain;

    Datalog datalog = new Datalog("MecanumAutoLog");
    boolean logData = true;
    public static ControlHub controlHub = new ControlHub();

    @Override
    public void runOpMode() {
        chassis = new Chassis(hardwareMap);

        // Pedro pathing init
        if (controlHub.getMacAddress().equals(Constants.PRIMARY_BOT)) {
            constants = new ConstantsCompetition();
        } else if (controlHub.getMacAddress().equals(Constants.SECONDARY_BOT)) {
            constants = new ConstantsDemo();
        } else {
            throw new RuntimeException("ControlHub MAC address did not match primary or secondary");
        }

        follower = constants.createFollower(hardwareMap);

        shooter = new Shooter(hardwareMap, "shooter", true);

        collector = hardwareMap.get(DcMotorEx.class, "collector");
        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setDirection(DcMotor.Direction.REVERSE);

        shooterHinge = hardwareMap.get(Servo.class, "shooterHinge");
        shooterHinge.setPosition(0.25);

        liftServo = hardwareMap.get(Servo.class, "liftServo");
        liftServo.setPosition(1.0);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        long delaySeconds = 0;

        limelight = new Limelight();
        limelight.init(hardwareMap, imu, telemetry);

        // Init
        do {
            limelight.readObelisk();

            telemetry.addData("Obelisk Bearing ", obeliskBearing);
            telemetry.addData("Obelisk Range ", obeliskDistance);

            telemetry.addLine();
            telemetry.addLine("--------------");
            telemetry.addLine();

            telemetry.addData("Press Up/Down dpad to adjust delay | Delay", delaySeconds);
            if (gamepad1.dpadUpWasReleased()) {
                delaySeconds++;
            }
            if (gamepad1.dpadDownWasReleased()) {
                delaySeconds--;
            }

            telemetry.addData("Press Right/Left dpad to toggle limited auto | Limited Auto", limitedAutoEnabled);
            telemetry.addData("Near auto enabled", nearAutoEnabled);

            if (gamepad1.dpadRightWasReleased() && !limitedAutoEnabled) {
                limitedAutoEnabled = true;
            }
            if (gamepad1.dpadLeftWasReleased() && limitedAutoEnabled) {
                limitedAutoEnabled = false;
            }

            if (gamepad1.xWasPressed() && gamepad1.right_bumper) {
                Blackboard.alliance = Blackboard.Alliance.BLUE;
            } else if (gamepad1.bWasPressed() && gamepad1.right_bumper) {
                Blackboard.alliance = Blackboard.Alliance.RED;
            }

            if (gamepad1.aWasPressed() && gamepad1.right_bumper) {
                nearAutoEnabled = true;
            } else if (gamepad1.yWasPressed() && gamepad1.right_bumper) {
                nearAutoEnabled = false;
            }

            telemetry.addData("Alliance", Blackboard.getAllianceAsString());
            telemetry.addData("Limelight team (alliance)", limelight.getTeam());
            telemetry.addData("Limelight obelisk", limelight.getObelisk());

            telemetry.addLine();
            telemetry.addLine("HOLD RB AND Press X to override alliance to BLUE");
            telemetry.addLine("HOLD RB AND Press B to override alliance to RED");

            telemetry.addLine("HOLD RB AND Press Y to override auto to FAR");
            telemetry.addLine("HOLD RB AND Press A to override auto to NEAR");

            telemetry.update();
        } while (opModeInInit());

        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            limelight.setTeam(24);
        } else {
            limelight.setTeam(20);
        }

        buildPaths(Blackboard.alliance); // Only build the paths once we press play(?)
        buildAutoStepChains();

        Pose correctedStartPose;
        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            if (nearAutoEnabled) {
                correctedStartPose = new Pose(144 - nearStartX, nearStartY, Math.toRadians((nearStartAngle - 90) * -1 + 90));
            } else {
                correctedStartPose = new Pose(144 - farStartX, farStartY, Math.toRadians((farStartAngle - 90) * -1 + 90));
            }
        } else {
            if (nearAutoEnabled) {
                correctedStartPose = new Pose(nearStartX, nearStartY, Math.toRadians(nearStartAngle));
            } else {
                correctedStartPose = new Pose(farStartX, farStartY, Math.toRadians(farStartAngle));
            }
        }
        follower.setStartingPose(correctedStartPose);

        runtime.reset();

        IterativeAutoStepChain activeIterativeAutoStepChain = farAutoStepChain;
        if (nearAutoEnabled) {
            activeIterativeAutoStepChain = nearAutoStepChain;
        }

        activeIterativeAutoStepChain.init();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            limelight.process();
            telemetry.addLine("----------");

            if (!activeIterativeAutoStepChain.done) {
                activeIterativeAutoStepChain.update(follower, collector, shooter, limelight, telemetry);
            }

            telemetry.update();
        }
    }

    void buildPaths(Blackboard.Alliance alliance) {
        // Blue alliance by default, unless it's proved that the alliance is red
        int sign = 1;
        if (alliance == Blackboard.Alliance.RED) {
            sign = -1;
        }

        // All poses are initially written as if we are on BLUE alliance; necessary values
        // are multiplied by horizontalSign to account for field symmetry
        inFrontOfBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((farStartX - 72) * sign + 72, farStartY),
                        new Pose((inFrontOfBalls1_x - 72) * sign + 72, inFrontOfBalls1_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((farStartAngle - 90) * sign + 90), Math.toRadians((inFrontOfBalls1_angle - 90) * sign + 90))
                .build();

        behindBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((inFrontOfBalls1_x - 72) * sign + 72, inFrontOfBalls1_y),
                        new Pose((behindBalls1_x - 72) * sign + 72, behindBalls1_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((inFrontOfBalls1_angle - 90) * sign + 90), Math.toRadians((behindBalls1_angle - 90) * sign + 90))
                .build();

        inFrontOfBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((farStartX - 72) * sign + 72, farStartY),
                        new Pose((inFrontOfBalls2_x - 72) * sign + 72, inFrontOfBalls2_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((farStartAngle - 90) * sign + 90), Math.toRadians((inFrontOfBalls2_angle - 90) * sign + 90))
                .build();

        behindBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((inFrontOfBalls2_x - 72) * sign + 72, inFrontOfBalls2_y),
                        new Pose((behindBalls2_x - 72) * sign + 72, behindBalls2_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((inFrontOfBalls2_angle - 90) * sign + 90), Math.toRadians((behindBalls2_angle - 90) * sign + 90))
                .build();

        inFrontOfBalls3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((farStartX - 72) * sign + 72, farStartY),
                        new Pose((inFrontOfBalls3_x - 72) * sign + 72, inFrontOfBalls3_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((farStartAngle - 90) * sign + 90), Math.toRadians((inFrontOfBalls3_angle - 90) * sign + 90))
                .build();

        behindBalls3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((inFrontOfBalls3_x - 72) * sign + 72, inFrontOfBalls3_y),
                        new Pose((behindBalls3_x - 72) * sign + 72, behindBalls3_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((inFrontOfBalls2_angle - 90) * sign + 90), Math.toRadians((behindBalls3_angle - 90) * sign + 90))
                .build();

        moveToFreeSpace = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((behindBalls1_x - 72) * sign + 72, behindBalls1_y),
                        new Pose((moveToFreeSpace_x - 72) * sign + 72, moveToFreeSpace_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((behindBalls1_angle - 90) * sign + 90), Math.toRadians((moveToFreeSpace_angle - 90) * sign + 90))
                .build();

        moveToFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((moveToFreeSpace_x - 72) * sign + 72, moveToFreeSpace_y),
                        new Pose((moveToFarShoot_x - 72) * sign + 72, moveToFarShoot_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((moveToFreeSpace_angle - 90) * sign + 90), Math.toRadians((moveToFarShoot_angle - 90) * sign + 90))
                .build();

        moveToNearShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((moveToFreeSpace_x - 72) * sign + 72, moveToFreeSpace_y),
                        new Pose((moveToNearShoot_x - 72) * sign + 72, moveToNearShoot_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians((moveToFreeSpace_angle - 90) * sign + 90), Math.toRadians((moveToNearShoot_angle - 90) * sign + 90))
                .build();
    }

    void buildAutoStepChains() {
        IterativeAutoStep moveToFarShootAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(moveToFarShoot)
                .setStartDelayMS(moveToFarShootDelayMS)
                .build();

        IterativeAutoStep moveToNearShootAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(moveToNearShoot)
                .setStartDelayMS(moveToNearShootDelayMS)
                .build();

        IterativeAutoStep moveToFreeSpaceAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(moveToFreeSpace)
                .build();

        IterativeAutoStep moveToInFrontOfBalls1AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(inFrontOfBalls1)
                .setCollectorOn(true)
                .setStartDelayMS(moveToInFrontOfBallsDelayMS)
                .build();

        IterativeAutoStep moveToBehindBalls1AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(behindBalls1)
                .setCollectorOn(true)
                .setStartDelayMS(moveToBehindBallsDelayMS)
                .setMaxPower(collectingMaxPower)
                .build();

        IterativeAutoStep moveToInFrontOfBalls2AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(inFrontOfBalls2)
                .setCollectorOn(true)
                .setStartDelayMS(moveToInFrontOfBallsDelayMS)
                .build();

        IterativeAutoStep moveToBehindBalls2AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(behindBalls2)
                .setCollectorOn(true)
                .setStartDelayMS(moveToBehindBallsDelayMS)
                .setMaxPower(collectingMaxPower)
                .build();

        IterativeAutoStep moveToInFrontOfBalls3AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(inFrontOfBalls3)
                .setCollectorOn(true)
                .setStartDelayMS(moveToInFrontOfBallsDelayMS)
                .build();

        IterativeAutoStep moveToBehindBalls3AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(behindBalls3)
                .setCollectorOn(true)
                .setStartDelayMS(moveToBehindBallsDelayMS)
                .setMaxPower(collectingMaxPower)
                .build();

        IterativeAutoStep shootThreeBallsAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.SHOOT)
                .setTargetShootCount(3)
                .setStartDelayMS(shootThreeBallsDelayMS)
                .build();


        // This is where you define the sequence of steps to be executed for each given auto

        farAutoStepChain = new IterativeAutoStepChain(
                collectorSpeed,
                new IterativeAutoStep[] {
                        moveToFarShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToInFrontOfBalls1AutoStep,
                        moveToBehindBalls1AutoStep,

                        moveToFarShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToInFrontOfBalls2AutoStep,
                        moveToBehindBalls2AutoStep,

                        moveToFarShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToInFrontOfBalls3AutoStep,
                        moveToBehindBalls3AutoStep,

                        moveToFarShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToFreeSpaceAutoStep,
                }
        );

        nearAutoStepChain = new IterativeAutoStepChain(
                collectorSpeed,
                new IterativeAutoStep[] {
                        moveToNearShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToInFrontOfBalls3AutoStep,
                        moveToBehindBalls3AutoStep,

                        moveToNearShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToInFrontOfBalls2AutoStep,
                        moveToBehindBalls2AutoStep,

                        moveToNearShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToInFrontOfBalls1AutoStep,
                        moveToBehindBalls1AutoStep,

                        moveToNearShootAutoStep,
                        shootThreeBallsAutoStep,

                        moveToFreeSpaceAutoStep,
                }
        );
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
        public Datalogger.GenericField turnPowerFactor = new Datalogger.GenericField("turnPowerFactor");

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
                            runTime, bearing, currentAngle, targetAngle, error, IMUAngle, turnPower, turnPowerFactor
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
