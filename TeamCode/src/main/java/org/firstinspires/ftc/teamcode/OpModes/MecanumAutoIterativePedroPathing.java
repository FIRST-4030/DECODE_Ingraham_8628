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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTag;
import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.ControlHub;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.IterativeAutoStep;
import org.firstinspires.ftc.teamcode.IterativeAutoStepChain;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsCompetition;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDemo;

@Configurable
@Autonomous(name="Mecanum Auto ITERATIVE PedroPathing", group="Linear OpMode")
public class MecanumAutoIterativePedroPathing extends LinearOpMode {

    // Pedro pathing constants (editable in panels)
    public static double start_x = 56, start_y = 8, start_angle = 90;
    public static double inFrontOfBalls1_x = 40, inFrontOfBalls1_y = 35,inFrontOfBalls1_angle = 0;

    public static double inFrontOfBalls2_x = 40, inFrontOfBalls2_y = 59 ,inFrontOfBalls2_angle = 0;
    public static double behindBalls1_x = 13, behindBalls1_y = 35, behindBalls1_angle = 0;
    public static double behindBalls2_x = 13, behindBalls2_y = 59, behindBalls2_angle = 0;
    public static double moveToFreeSpace_x = 50, moveToFreeSpace_y = 35, moveToFreeSpace_angle = 0;
    public static double moveToFarShoot_x = 60, moveToFarShoot_y = 11, moveToFarShoot_angle = 111;
    Pose startPose = new Pose(56, 8, Math.toRadians(90));

    Chassis chassis;
    Constants constants;
    DcMotorEx collector;
    Shooter shooter;
    Servo shooterHinge;

    ElapsedTime runtime = new ElapsedTime();

    AprilTag aprilTags;

    Servo liftServo;

    ElapsedTime collectorTime = new ElapsedTime();

    double obeliskBearing, obeliskDistance;
    double collectorSpeed = 0.5;

    boolean limitedAutoEnabled = false;

    IMU imu;

    Follower follower;
    PathChain inFrontOfBalls1, behindBalls1, inFrontOfBalls2, behindBalls2, moveToFreeSpace, moveToFarShoot;
    IterativeAutoStepChain farAutoStepChain;


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
        follower.setStartingPose(startPose);   //set your starting pose

        buildPaths(Blackboard.alliance); // Build the paths once we know the alliance
        buildAutoStepChains();

        shooter = new Shooter(hardwareMap, "shooter", true);

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
        long delaySeconds = 0;

        // Init
        do {
            aprilTags.scanField(telemetry);
            obeliskBearing = aprilTags.getObeliskBearing();
            obeliskDistance = aprilTags.getObeliskRange();

            telemetry.addData("Obelisk Bearing ", obeliskBearing);
            telemetry.addData("Obelisk Range ", obeliskDistance);

            if (aprilTags.getObeliskRange() > 100) telemetry.addData("Field Position", "Far");
            if (aprilTags.getObeliskRange() < 100) telemetry.addData("Field Position", "Close");

            if (obeliskBearing > 0) {
                if (aprilTags.getObeliskRange() > 100) {
                    Blackboard.alliance = Blackboard.Alliance.RED;
                }
                else {
                    Blackboard.alliance = Blackboard.Alliance.BLUE;
                }
            }
            if (obeliskBearing < 0 && obeliskBearing > -30) {
                if (aprilTags.getObeliskRange() > 100) {
                    Blackboard.alliance = Blackboard.Alliance.BLUE;
                } else {
                    Blackboard.alliance = Blackboard.Alliance.RED;
                }
            }

            telemetry.addData("Range to Obelisk AprilTag", aprilTags.getObeliskRange());

            telemetry.addLine();
            telemetry.addLine("--------------");
            telemetry.addLine();

            telemetry.addData("Press Y (increase delay), A (decrease delay) | Delay", delaySeconds);
            if (gamepad1.yWasReleased()) {
                delaySeconds++;
            }
            if (gamepad1.aWasReleased()) {
                delaySeconds--;
            }

            telemetry.addData("Press Right/Left dpad to toggle limited auto | Limited Auto", limitedAutoEnabled);
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

            telemetry.addData("Alliance", Blackboard.getAllianceAsString());
            telemetry.addLine();
            telemetry.addLine("--- IF NO ALLIANCE IS DETECTED (FOR SOME REASON) ---");
            telemetry.addLine("HOLD RB AND Press X to override alliance to BLUE");
            telemetry.addLine("HOLD RB AND Press B to override alliance to RED");

            telemetry.update();
        } while (opModeInInit());

        farAutoStepChain.init(follower);

        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            startPose = new Pose(-start_x, 8, Math.toRadians(90));
            follower.setStartingPose(startPose);   //set your starting pose
        }


        runtime.reset();
        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            farAutoStepChain.update(follower, collector, telemetry);
//            doFarAutoLinear(Blackboard.alliance);
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
                        new Pose((start_x - 72) * sign + 72, start_y),
                        new Pose((inFrontOfBalls1_x - 72) * sign + 72, inFrontOfBalls1_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(start_angle * sign), Math.toRadians((inFrontOfBalls1_angle - 90) * sign + 90))
                .build();

        behindBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((inFrontOfBalls1_x - 72) * sign + 72, inFrontOfBalls1_y),
                        new Pose((behindBalls1_x - 72) * sign + 72, behindBalls1_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(inFrontOfBalls1_angle * sign), Math.toRadians((behindBalls1_angle - 90) * sign + 90))
                .build();

        inFrontOfBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((start_x - 72) * sign + 72, start_y),
                        new Pose((inFrontOfBalls2_x - 72) * sign + 72, inFrontOfBalls2_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(start_angle * sign), Math.toRadians((inFrontOfBalls2_angle - 90) * sign + 90))
                .build();

        behindBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((inFrontOfBalls2_x - 72) * sign + 72, inFrontOfBalls2_y),
                        new Pose((behindBalls2_x - 72) * sign + 72, behindBalls2_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(inFrontOfBalls2_angle * sign), Math.toRadians((behindBalls2_angle - 90) * sign + 90))
                .build();

        moveToFreeSpace = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((behindBalls1_x - 72) * sign + 72, behindBalls1_y),
                        new Pose((50. - 72) * sign + 72, 50.)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(behindBalls1_angle * sign), Math.toRadians((moveToFreeSpace_angle - 90) * sign + 90))
                .build();

        moveToFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((moveToFreeSpace_x - 72) * sign + 72, moveToFreeSpace_y),
                        new Pose((moveToFarShoot_x - 72) * sign + 72, moveToFarShoot_y)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToFreeSpace_angle * sign), Math.toRadians((moveToFarShoot_angle - 90) * sign + 90))
                .build();
    }

    void buildAutoStepChains() {
        IterativeAutoStep moveToFarShootAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(moveToFarShoot)
                .setStartDelayMS(50)
                .build();

        IterativeAutoStep moveToFreeSpaceAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(moveToFreeSpace)
                .build();

        IterativeAutoStep moveToInFrontOfBalls1AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(inFrontOfBalls1)
                .setStartDelayMS(700)
                .build();

        IterativeAutoStep moveToBehindBalls1AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(behindBalls1)
                .setStartDelayMS(50)
                .build();

        IterativeAutoStep moveToInFrontOfBalls2AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(inFrontOfBalls2)
                .setStartDelayMS(700)
                .build();

        IterativeAutoStep moveToBehindBalls2AutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(behindBalls2)
                .setStartDelayMS(50)
                .build();

        moveToFreeSpaceAutoStep = new IterativeAutoStep.Builder()
                .setStepType(IterativeAutoStep.StepType.MOVE)
                .setPathChain(moveToFreeSpace)
                .build();


        // This is where you define the sequence of steps to be executed for each given auto
        farAutoStepChain = new IterativeAutoStepChain(
                34.0,
                new IterativeAutoStep[] {
                        moveToFarShootAutoStep,

                        moveToInFrontOfBalls1AutoStep,
                        moveToBehindBalls1AutoStep,

                        moveToFarShootAutoStep,

                        moveToInFrontOfBalls2AutoStep,
                        moveToBehindBalls2AutoStep,

                        moveToFarShootAutoStep,
                }
        );
    }

    private void moveForward(double power, double mseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < mseconds) {
            chassis.frontLeftDrive.setPower(power);
            chassis.backLeftDrive.setPower(power);
            chassis.frontRightDrive.setPower(power);
            chassis.backRightDrive.setPower(power);
        }

        stopMotors();
    }

    private void stopMotors() {
        chassis.frontLeftDrive.setPower(0);
        chassis.backLeftDrive.setPower(0);
        chassis.frontRightDrive.setPower(0);
        chassis.backRightDrive.setPower(0);
    }

    public void shootShooter(double velocity) {
        // shooter.targetVelocity = (aprilTags.distanceToGoal + 202.17) / 8.92124;
        shooter.targetVelocity = velocity;
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

            float turnPowerFactor = 1;
            if (Math.abs(error) < 10) {
                turnPowerFactor = 0.5f;
            }

            turnPower = Kp * error + Kd * derivative * turnPowerFactor;

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

            chassis.frontLeftDrive.setPower(-turnPower);
            chassis.backLeftDrive.setPower(-turnPower);
            chassis.frontRightDrive.setPower(turnPower);
            chassis.backRightDrive.setPower(turnPower);

            if (logData) {
                datalog.runTime.set(runtime.seconds());
                datalog.bearing.set(aprilTags.getBearing());
                datalog.targetAngle.set(targetAngle);
                datalog.currentAngle.set(currentAngle);
                datalog.error.set(error);
                datalog.turnPower.set(turnPower);
                datalog.IMUAngle.set(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                datalog.turnPowerFactor.set(turnPowerFactor);

                datalog.writeLine();
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
