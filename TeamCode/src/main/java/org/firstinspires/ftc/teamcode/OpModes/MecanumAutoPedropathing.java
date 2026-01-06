
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
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsCompetition;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDemo;

@Configurable
@Autonomous(name="Mecanum Auto PEDROPATHING YIPPEE", group="Linear OpMode")
public class MecanumAutoPedropathing extends LinearOpMode {

    // Pedro pathing constants (editable in panels)
    public static double start_x = 56, start_y = 8, start_angle = 90;
    public static double inFrontOfBalls1_x = 40, inFrontOfBalls1_y = 35,inFrontOfBalls1_angle = 0;
    public static double behindBalls1_x = 13, behindBalls1_y = 35, behindBalls1_angle = 0;
    public static double moveToFreeSpace_x = 50, moveToFreeSpace_y = 35,moveToFreeSpace_angle = 0;
    public static double moveToFarShoot_x = 60, moveToFarShoot_y = 11,moveToFarShoot_angle = 110;

    Chassis chassis;
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
    PathChain InFrontOfBalls1, BehindBalls1, MoveToFreeSpace, MoveToFarShoot;

    // The order of values listed in Options is irrelevant
    enum Options { STOP, Do_InFrontOfBalls1, Do_BehindBalls1, Do_MoveToFreeSpace, Do_MoveToFarShoot }
    PedroPathingDemoAuto.Options option;

    boolean doAutonomous = false;

    Datalog datalog = new Datalog("MecanumAutoLog");
    boolean logData = true;

    @Override
    public void runOpMode() {
        chassis = new Chassis(hardwareMap);

        // Pedro pathing init
        Pose startPose = new Pose(56, 8, Math.toRadians(90));

        follower = new ConstantsDemo().createFollower(hardwareMap);
        follower.setStartingPose(startPose);   //set your starting pose

        buildPaths();

        doAutonomous = true;
        Options option = Options.Do_InFrontOfBalls1;   // Define the first action in the path

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
        long delaySeconds=0;

        // Init
        do {
            aprilTags.scanField(telemetry);
            obeliskBearing = aprilTags.getObeliskBearing();
            obeliskDistance = aprilTags.getObeliskRange();

            telemetry.addData("!!! Obelisk Bearing ", obeliskBearing);
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

            telemetry.addData("Press X (increase delay), Y (decrease delay) | Delay", delaySeconds);
            if (gamepad1.xWasReleased()) {
                delaySeconds++;
            }
            if (gamepad1.yWasReleased()) {
                delaySeconds--;
            }

            telemetry.addData("Press Right/Left bumper to toggle limited auto | Limited Auto", limitedAutoEnabled);
            if (gamepad1.rightBumperWasReleased() && !limitedAutoEnabled) {
                limitedAutoEnabled = true;
            }
            if (gamepad1.leftBumperWasReleased() && limitedAutoEnabled) {
                limitedAutoEnabled = false;
            }

            telemetry.addData("Alliance", Blackboard.getAllianceAsString());

            telemetry.update();
        } while (opModeInInit());

        runtime.reset();
        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (doAutonomous) {
                if (limitedAutoEnabled) {
                    runLimitedAuto();
                }
                else {
                    if (obeliskDistance > 100) {
                        runFromFar();
                    } else {
                        runFromClose();
                    }
                }
            }
            else {
                break;
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

    private void runFromFar() {
        imu.resetYaw();
        double shootingVelocity = 34.0;

        follower.update();

        switch (option) {
            case Do_InFrontOfBalls1:
                if (!follower.isBusy()) {
                    follower.followPath(InFrontOfBalls1);
                    option = PedroPathingDemoAuto.Options.Do_BehindBalls1;
                }
                break;
            case Do_BehindBalls1:
                if (!follower.isBusy()) {
                    follower.followPath(BehindBalls1);
                    option = PedroPathingDemoAuto.Options.Do_MoveToFreeSpace;
                }
                break;
            case Do_MoveToFreeSpace:
                if (!follower.isBusy()) {
                    follower.followPath(MoveToFreeSpace);
                    option = PedroPathingDemoAuto.Options.Do_MoveToFarShoot;
                }
                break;
            case Do_MoveToFarShoot:
                if (!follower.isBusy()) {
                    follower.followPath(MoveToFarShoot);
                    option = PedroPathingDemoAuto.Options.STOP;
                }
                break;
            case STOP:
                if (!follower.isBusy()) {
                    doAutonomous = false;
                }
                break;
        }



        //if (redSide) {
        if (Blackboard.alliance == Blackboard.Alliance.RED) {

//            // Aim and shoot the three initial artifacts
//            moveForward(0.5, 75);
//
//            rotateTo(aprilTags.getBearing() + 5);
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            stopShooter();
//
//            // Because this is RED AUTO, rotate back to face straight forward and reset yaw
//            rotateTo(10);
//            imu.resetYaw();
//
//            // Move and collect three artifacts in a line
//            moveForward(0.5, 915);
//            rotateTo(90);
//
//            collector.setPower(collectorSpeed);
//            moveForward(-0.2, 4000);
//            collectorTime.reset();
//            while (collectorTime.milliseconds() < 1000) collector.setPower(collectorSpeed);
//            collector.setPower(0);
//
//            // Move back to line up and shoot
//            moveForward(0.25, 2750);
//
//            rotateTo(0);
//            moveForward(-0.25, 1500);
//
//            rotateTo(-10); // ?????
//
//            // Aim and shoot the three artifacts we collected!
//            rotateTo(aprilTags.getBearing() - 8);
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            stopShooter();
//
//            // Move off the white line
//            moveForward(0.5, 800);
        }
        else if(Blackboard.alliance == Blackboard.Alliance.BLUE) {
//            // Aim and shoot the three initial artifacts
//            moveForward(0.5, 250);
//
//            rotateTo (aprilTags.getBearing() - 3);
//
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            stopShooter();
//
//            // Move and collect three artifacts in a line
//            rotateTo(0);
//            moveForward(0.5, 450);
//            rotateTo(-90);
//
//            collector.setPower(collectorSpeed);
//            moveForward(-0.2, 4375);
//            collectorTime.reset();
//            while (collectorTime.milliseconds() < 1000) collector.setPower(collectorSpeed);
//            collector.setPower(0);
//
//            // Move back to line up and shoot
//            moveForward(0.25, 2250);
//
//            rotateTo(0);
//            moveForward(-0.25, 1500);
//
//
//            // Aim and shoot the three artifacts we collected!
//            rotateTo(aprilTags.getBearing() - 5);
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            shootShooter(shootingVelocity);
//            stopShooter();
//
//            // Move off the white line
//            moveForward(0.5, 800);
        }
    }

    private void runFromClose() {
        double shootingVelocity = 29.0;

        // Remember this for close auto; our "home base" angle of zero is parallel to the goals:
        imu.resetYaw();

        if (Blackboard.alliance == Blackboard.Alliance.BLUE) {
            // Move to be in front of the goal we're next to
            rotateTo(130);
            moveForward(-0.5, 1550);

            // SHOOT!
            shootShooter(shootingVelocity);
            shootShooter(shootingVelocity);
            shootShooter(shootingVelocity);
            stopShooter();

            // Line up and collect three artifacts in a line
            rotateTo(-135);
            moveForward(0.5, 570);
            rotateTo(0);

            collector.setPower(collectorSpeed);
            moveForward(-0.2, 2850);
            collectorTime.reset();
            while (collectorTime.milliseconds() < 1000) collector.setPower(collectorSpeed);
            collector.setPower(0);
        }
        else {
            // Move to be in front of the goal we're next to
            rotateTo(-130);
            moveForward(-0.5, 1550);

            // SHOOT!
            shootShooter(shootingVelocity);
            shootShooter(shootingVelocity);
            shootShooter(shootingVelocity);
            stopShooter();

            // Line up and collect three artifacts in a line
            rotateTo(135);
            moveForward(0.5, 570);
            rotateTo(0);

            collector.setPower(collectorSpeed);
            moveForward(-0.2, 2850);
            collectorTime.reset();
            while (collectorTime.milliseconds() < 1000) collector.setPower(collectorSpeed);
            collector.setPower(0);
        }

        //unused auto bits
//        moveForward(0.25, 2550);
//
//        rotateTo(39);
//
//        shootShooter();
//        shootShooter();
//        shootShooter();
//        stopShooter();
//
//        rotateTo(-220);
//
//        moveForward(0.5, 700);
    }

    private void runLimitedAuto() {
        double shootingVelocity = 34.0;

        // Aim and shoot three artifacts
        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            rotateTo(aprilTags.getBearing() + 5);
        } else {
            rotateTo(aprilTags.getBearing() - 3);
        }

        shootShooter(shootingVelocity);
        shootShooter(shootingVelocity);
        shootShooter(shootingVelocity);
        stopShooter();

        // Move off the white line, that's it!
        rotateTo(0);
        moveForward(0.5, 700);
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

//    private void rotate (double milliseconds, int reverse) {
//        ElapsedTime turnTimer = new ElapsedTime();
//        turnTimer.reset();
//        int leftSidepos;
//
//        if (blueSide) {
//            leftSidepos = reverse;
//        }
//        else {
//            leftSidepos = -reverse;
//        }
//
//        while (turnTimer.milliseconds() < milliseconds) {
//            frontLeftDrive.setPower(leftSidepos * -0.5);
//            backLeftDrive.setPower(leftSidepos * -0.5);
//            frontRightDrive.setPower(leftSidepos * 0.5);
//            backRightDrive.setPower(leftSidepos * 0.5);
//        }
//
//        stopMotors();
//    }

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
