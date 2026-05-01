package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Configurable
@Autonomous(name="Mecanum Limelight - Refactoring in progress", group="Linear OpMode")
public class MecanumAuto_Limelight extends LinearOpMode {

    public static int polyRangeCrossover = 80;
    public static int polyVeloBaseFar = 19;
    public static int polyVeloBaseNear = 29;
    public static double polyVeloBaseRangeFactor = 0.125;

    public static long moveToInsideRowsMS = 250;
    public static long moveToOutsideRowsMS = 0;
    public static long moveToFarShootDelayMS = 0;
    public static long moveToNearShootDelayMS = 0;
    public static long shootThreeBallsDelayMS = 0;

    public static double collectorSpeed = 0.45;
    public static float collectingMaxPower = 0.3f;

    // PedroPathing poses
    public static Pose farStartPose = new Pose(56, 8, 90);
    public static Pose nearStartPose = new Pose(26, 122, 315);

    public static Pose insideRow1Pose = new Pose(50, 35, 0);
    public static Pose insideRow2Pose = new Pose(50, 59, 0);
    public static Pose insideRow3Pose = new Pose(50, 83, 0);

    public static Pose outsideRow1Pose = new Pose(23, 35, 0);
    public static Pose outsideRow2Pose = new Pose(23, 59, 0);
    public static Pose outsideRow3Pose = new Pose(23, 83, 0);

    public static Pose farShootPose = new Pose(555, 16, 110);
    public static Pose nearShootPose = new Pose(48, 96, 135);
    public static Pose freeSpacePose = new Pose(20, 8, 90);

    Follower follower;
    PathChain
            goInsideRow1,
            goInsideRow2,
            goInsideRow3,

            goOutsideRow1,
            goOutsideRow2,
            goOutsideRow3,

            goFarShoot,
            goNearShoot,
            goFreeSpace;
    IterativeAutoStepChain farAutoStepChain, nearAutoStepChain;

    Chassis chassis;
    Constants constants;
    DcMotorEx collector;
    Shooter shooter;
    Servo shooterHinge;
    Servo liftServo;
    IMU imu;
    Limelight limelight;

    ElapsedTime runtime = new ElapsedTime();

    // double obeliskBearing, obeliskDistance;

    boolean limitedAutoEnabled = false;
    boolean nearAutoEnabled = false;

    // Datalog datalog = new Datalog("MecanumAutoLog");
    // boolean logData = true;

    public static ControlHub controlHub = new ControlHub();

    FieldManager panelsFieldManager = PanelsField.INSTANCE.getField();
    GamepadManager panelsGamepad = PanelsGamepad.INSTANCE.getFirstManager();

    @Override
    public void runOpMode() {
        chassis = new Chassis(hardwareMap);

        // PedroPathing constants init (TEMPORARY, unstable because of Mac Address unpredictability)
        if (controlHub.getNetworkName().equals(Constants.PRIMARY_BOT_NETWORK_NAME)) {
            constants = new ConstantsCompetition();
        } else if (controlHub.getNetworkName().equals(Constants.PRIMARY_BOT_NETWORK_NAME)) {
            constants = new ConstantsDemo();
        }

        follower = constants.createFollower(hardwareMap);

        shooter = new Shooter(hardwareMap, "shooter", true);
        shooter.setVeloParameters(polyRangeCrossover, polyVeloBaseFar, polyVeloBaseNear, polyVeloBaseRangeFactor);

        collector = hardwareMap.get(DcMotorEx.class, "collector");
        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setDirection(DcMotor.Direction.REVERSE);

        shooterHinge = hardwareMap.get(Servo.class, "shooterHinge");
        shooter.putHingeDown();

        liftServo = hardwareMap.get(Servo.class, "liftServo");
        liftServo.setPosition(1);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight = new Limelight();
        limelight.init(hardwareMap, imu, telemetry);

        // Init
        do {
            limelight.getTagLocations("Red", imu);
            limelight.getTagLocations("Blue", imu);
            limelight.readObelisk();

            // telemetry.addData("Obelisk Bearing ", obeliskBearing);
            // telemetry.addData("Obelisk Range ", obeliskDistance);

            telemetry.addLine();
            telemetry.addLine("--------------");
            telemetry.addLine();

            // Limited auto hasn't been implemented for this robot yet
            // telemetry.addData("Press Right/Left dpad to toggle limited auto | Limited Auto", limitedAutoEnabled);

            // if (gamepad1.dpadRightWasReleased() && !limitedAutoEnabled) {
            //     limitedAutoEnabled = true;
            // }
            // if (gamepad1.dpadLeftWasReleased() && limitedAutoEnabled) {
            //     limitedAutoEnabled = false;
            // }

            telemetry.addData("Near auto enabled", nearAutoEnabled);

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
            telemetry.addLine("Hold RB and Press X to override alliance to BLUE");
            telemetry.addLine("Hold RB and Press B to override alliance to RED");

            telemetry.addLine("Hold RB and Press Y to override auto to FAR");
            telemetry.addLine("Hold RB and Press A to override auto to NEAR");

            telemetry.update();
        } while (opModeInInit());

        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            limelight.setTeam(24);
        } else {
            limelight.setTeam(20);
        }

        buildPaths(Blackboard.alliance); // Only build the paths once we press play(?)
        buildAutoStepChains();

        follower.setStartingPose(poseAutoFlip(farStartPose));
        if (nearAutoEnabled) {
            follower.setStartingPose(poseAutoFlip(nearStartPose));
        }

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
                activeIterativeAutoStepChain.update(follower, collector, shooter, limelight, telemetry, chassis);
            }

            drawPanelsField();
            telemetry.update();
        }
    }

    void drawPanelsField() {
        drawBotPoseToPanelsField();
        panelsFieldManager.update();
    }

    void drawBotPoseToPanelsField() {
        panelsFieldManager.setStyle(new Style("none", "white", 1.5));
        panelsFieldManager.moveCursor(follower.getPose().getX(), follower.getPose().getY());
        panelsFieldManager.setCursorHeading(follower.getHeading());
        panelsFieldManager.rect(10, 10);
    }

    void buildPaths(Blackboard.Alliance alliance) {
        // Blue alliance by default, unless it's proved that the alliance is red
        boolean doMirror = Blackboard.alliance == Blackboard.Alliance.RED;

        // All poses are initially written as if we are on BLUE alliance; necessary values
        // are multiplied by horizontalSign to account for field symmetry
        goInsideRow1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(farStartPose),
                        poseAutoFlip(insideRow1Pose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(farStartPose).getHeading(), poseAutoFlip(insideRow1Pose).getHeading())
                .build();

        goInsideRow2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(farStartPose),
                        poseAutoFlip(insideRow2Pose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(farStartPose).getHeading(), poseAutoFlip(insideRow2Pose).getHeading())
                .build();

        goInsideRow3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(farStartPose),
                        poseAutoFlip(insideRow3Pose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(farStartPose).getHeading(), poseAutoFlip(insideRow3Pose).getHeading())
                .build();

        goOutsideRow1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(insideRow1Pose),
                        poseAutoFlip(outsideRow1Pose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(insideRow1Pose).getHeading(), poseAutoFlip(outsideRow1Pose).getHeading())
                .build();

        goOutsideRow2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(insideRow2Pose),
                        poseAutoFlip(outsideRow2Pose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(insideRow2Pose).getHeading(), poseAutoFlip(outsideRow2Pose).getHeading())
                .build();

        goOutsideRow3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(insideRow3Pose),
                        poseAutoFlip(outsideRow3Pose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(insideRow3Pose).getHeading(), poseAutoFlip(outsideRow3Pose).getHeading())
                .build();

        goFreeSpace = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(farShootPose),
                        poseAutoFlip(freeSpacePose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(farShootPose).getHeading(), poseAutoFlip(freeSpacePose).getHeading())
                .build();

        goFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(insideRow1Pose),
                        poseAutoFlip(farShootPose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(insideRow1Pose).getHeading(), poseAutoFlip(farShootPose).getHeading())
                .build();

        goNearShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAutoFlip(insideRow1Pose),
                        poseAutoFlip(nearShootPose)
                ))
                .setLinearHeadingInterpolation(poseAutoFlip(insideRow1Pose).getHeading(), poseAutoFlip(nearShootPose).getHeading())
                .build();
    }

    void buildAutoStepChains() {
        IterativeAutoStep moveToFarShootAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goFarShoot)
                .setStartDelayMS(moveToFarShootDelayMS)
                .build();

        IterativeAutoStep moveToNearShootAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goNearShoot)
                .setStartDelayMS(moveToNearShootDelayMS)
                .build();

        IterativeAutoStep moveToFreeSpaceAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goFreeSpace)
                .build();

        IterativeAutoStep moveToInFrontOfBalls1AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goInsideRow1)
                .setCollectorOn(true)
                .setStartDelayMS(moveToInsideRowsMS)
                .build();

        IterativeAutoStep moveToBehindBalls1AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goOutsideRow1)
                .setCollectorOn(true)
                .setStartDelayMS(moveToOutsideRowsMS)
                .setMaxPower(collectingMaxPower)
                .build();

        IterativeAutoStep moveToInFrontOfBalls2AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goInsideRow2)
                .setCollectorOn(true)
                .setStartDelayMS(moveToInsideRowsMS)
                .build();

        IterativeAutoStep moveToBehindBalls2AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goOutsideRow2)
                .setCollectorOn(true)
                .setStartDelayMS(moveToOutsideRowsMS)
                .setMaxPower(collectingMaxPower)
                .build();

        IterativeAutoStep moveToInFrontOfBalls3AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goInsideRow3)
                .setCollectorOn(true)
                .setStartDelayMS(moveToInsideRowsMS)
                .build();

        IterativeAutoStep moveToBehindBalls3AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(goOutsideRow3)
                .setCollectorOn(true)
                .setStartDelayMS(moveToOutsideRowsMS)
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

//                        moveToFarShootAutoStep,
//                        shootThreeBallsAutoStep,

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

//                        moveToInFrontOfBalls2AutoStep,
//                        moveToBehindBalls2AutoStep,
//
//                        moveToNearShootAutoStep,
//                        shootThreeBallsAutoStep,
//
//                        moveToInFrontOfBalls1AutoStep,
//                        moveToBehindBalls1AutoStep,
//
//                        moveToNearShootAutoStep,
//                        shootThreeBallsAutoStep,

                        moveToFreeSpaceAutoStep,
                }
        );
    }

    Pose poseAutoFlip(Pose pose) {
        boolean doMirror = (Blackboard.alliance == Blackboard.Alliance.RED);

        if (doMirror) {
            return pose.mirror();
        }
        return pose;
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
