package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.Shooter;

@Configurable
@TeleOp(name = "Mecanum Teleop Limelight", group = "Robot")
public class MecanumTeleop_Limelight extends OpMode {
    public static double collectorSpeed = 0.45;

    public static int polyRangeCrossover = 80;
    public static int polyVeloBaseFar = 19;
    public static int polyVeloBaseNear = 29;
    public static double polyVeloBaseRangeFactor = 0.125;
    public static double aimLeniencyDegrees = 5;

    Chassis chassis;
    DcMotorEx collector;
    Shooter shooter;
    Servo liftServo;
    Limelight limelight;
    IMU imu;

    boolean targetInView;
    boolean collectorOn = false;

    int currentShootCount = 0;
    int targetShootCount = 1;
    boolean isShooting = false;
    boolean reachedSpeed = false;
    ElapsedTime shotTimer = new ElapsedTime();

    public static double SHOOTER_HINGE_LIFT_DURATION_MS = 400;
    public static double SHOT_DURATION_MS = 800;

    @Override
    public void init() {
        Blackboard.alliance = Blackboard.Alliance.BLUE;

        chassis = new Chassis(hardwareMap);

        shooter=new Shooter(hardwareMap,"shooter",true);
        shooter.setVeloParameters(polyRangeCrossover, polyVeloBaseFar, polyVeloBaseNear, polyVeloBaseRangeFactor);

        collector = hardwareMap.get(DcMotorEx.class, "collector");

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collector.setDirection(DcMotor.Direction.REVERSE);

        liftServo = hardwareMap.get(Servo.class, "liftServo");
        liftServo.setPosition(1.0);  // Up with Axon Max is 1.0,  was 0.0 with Savox

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
    }

    @Override
    public void init_loop() {
        limelight.readObelisk();

        if (gamepad1.xWasPressed() && gamepad1.right_bumper) {
            Blackboard.alliance = Blackboard.Alliance.BLUE;
        } else if (gamepad1.bWasPressed() && gamepad1.right_bumper) {
            Blackboard.alliance = Blackboard.Alliance.RED;
        }

        telemetry.addData("Pad 1, Left Bumper", "Very Slow Drive");
        telemetry.addData("Pad 1, Right Bumper", "Slow Drive");
        telemetry.addData("Pad 1, A", "Raise Robot");
        telemetry.addData("Pad 1, Y", "Lower Robot");
        telemetry.addData("--", "--");
        telemetry.addData("Pad 2, Left Bumper", "Shoot 1");
        telemetry.addData("Pad 2, Right Bumper", "Shoot 3");
        telemetry.addData("Pad 2, B", "Collector On/Off");
        telemetry.addData("Pad 2, X", "Collector Reverse");

        telemetry.addData("Obelisk", limelight.getObelisk());
        telemetry.addData("Alliance", Blackboard.getAllianceAsString());
        telemetry.addLine("Hold RB and Press X to override alliance to BLUE");
        telemetry.addLine("Hold RB and Press B to override alliance to RED");

        telemetry.update();
    }

    public void start() {
        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            limelight.setTeam(24);
        }
        else if (Blackboard.alliance == Blackboard.Alliance.BLUE) {
            limelight.setTeam(20);
        }
        collector.setPower(collectorSpeed);
        collectorOn = true;
    }

    @Override
    public void loop() {
        targetInView = limelight.process();
        shooter.overridePower();

        telemetry.addData("Target is in view:", targetInView);
        telemetry.addData("Shooter Current Velocity", shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity", shooter.targetVelocity);
        telemetry.addData("Distance to Target", limelight.getRange());

        //Gamepad 1
        if (gamepad1.start) {
            imu.resetYaw();
        }

        //Slow Drive
        if (gamepad1.rightBumperWasPressed()) {
            chassis.setMaxSpeed(0.4);
        }
        if (gamepad1.leftBumperWasReleased()) {
            chassis.setMaxSpeed(1.0);
        }

        //Precision Drive
        if (gamepad1.leftBumperWasPressed()) {
            chassis.setMaxSpeed(0.2);
        }
        if (gamepad1.rightBumperWasReleased()) {
            chassis.setMaxSpeed(1.0);
        }

//        //Lift Servo Controls
//        if (gamepad1.yWasPressed()) {
//            shooter.targetVelocity = 0;
//            collector.setPower(0.0);
//            collectorOn = false;
//            liftServo.setPosition(1.0);
//        }
//        if (gamepad1.aWasPressed()) {
//            shooter.targetVelocity = 0;
//            collector.setPower(0.0);
//            collectorOn = false;
//            liftServo.setPosition(0.0);
//        }
//
//        //Gamepad 2
//        if (gamepad1.start) {
//            imu.resetYaw();
//        }

        //Collector Controls
        if (gamepad1.bWasReleased()) {
            if (!collectorOn) {
                collector.setPower(collectorSpeed);
                collectorOn = true;
            }
//            else {
//                collector.setPower(0.0);
//                collectorOn = false;
//            }
        }

        if (gamepad1.xWasPressed()) {
            collector.setPower(-collectorSpeed);
        }
        if (gamepad1.xWasReleased()) {
            collector.setPower(collectorSpeed);
            collectorOn = false;
        }

        handleShooting();

        if (!isShooting) {
            chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            chassis.drive(0, 0, 0);
        }

        double limelightGetTx = limelight.getTx();
        telemetry.addData("TX", limelightGetTx);
        telemetry.addData("Within leniency range", isWithinLeniencyRange());

        telemetry.addData("Current Shoot Count", currentShootCount);
        telemetry.addData("Collector Current Power:", collector.getVelocity());
        telemetry.addData("Collector Target Power", collectorSpeed);
        telemetry.addData("Shooter Current Velocity:", shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity: ", shooter.targetVelocity);
        telemetry.addData("Get Shooter Velocity", shooter.getShooterVelo(limelight));
        telemetry.update();
    }

    public void handleShooting() {
        if (gamepad1.left_trigger > 0.5 && !isShooting && isWithinLeniencyRange()) {
            isShooting = true;
            currentShootCount = 0;
            targetShootCount = 1;
            reachedSpeed = false;
            shotTimer.reset();
        }

        if (gamepad1.right_trigger > 0.5 && !isShooting && isWithinLeniencyRange()) {
            isShooting = true;
            currentShootCount = 0;
            targetShootCount = 3;
            reachedSpeed = false;
            shotTimer.reset();
        }

        if (isShooting) {
            shooter.setTargetVelocity(shooter.getShooterVelo(limelight));
            shooter.overridePower();

            collector.setPower(-collectorSpeed);
            collectorOn = false;

            if (shooter.atSpeed()) {
                reachedSpeed = true;
            }

            if (!reachedSpeed) {
                shotTimer.reset();
                return;
            }

            if (shotTimer.milliseconds() < SHOOTER_HINGE_LIFT_DURATION_MS) {
                shooter.putHingeDown();
            } else if (shotTimer.milliseconds() < SHOT_DURATION_MS) {
                shooter.putHingeUp();
            } else {
                currentShootCount ++;
                reachedSpeed = false;
                shotTimer.reset();
                if (currentShootCount == targetShootCount) {
                    isShooting = false;
                    shooter.stopShooter();
                    shooter.putHingeDown();
                    collector.setPower(collectorSpeed);
                    collectorOn = true;
                }
            }
        } else {
//            if (gamepad1.y) {
//                shooter.setTargetVelocity(35);
//                shooter.overridePower();
//            } else {
//                shooter.stopShooter();
//            }
        }
    }

    public boolean isWithinLeniencyRange() {
        return limelight.hasResults() && Math.abs(limelight.getTx()) <= aimLeniencyDegrees;
    }
}