package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.Shooter;

@Configurable
@TeleOp(name = "Zeven's Learning OP Mode", group = "Robot")
public class MecanumTeleop_Zeven extends OpMode {
    public static double collectorSpeed = 0.45;

    public static int polyRangeCrossover = 80;
    public static int polyVeloBaseFar = 19;
    public static int polyVeloBaseNear = 29;
    public static double polyVeloBaseRangeFactor = 0.125;
    public static double aimLeniencyDegrees = 3;
    public DigitalChannel green, red;

    Shooter shooter;
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

        shooter=new Shooter(hardwareMap,"shooter",true);
        shooter.setVeloParameters(polyRangeCrossover, polyVeloBaseFar, polyVeloBaseNear, polyVeloBaseRangeFactor);
        imu = hardwareMap.get(IMU.class, "imu");
        green = hardwareMap.get(DigitalChannel.class, "green");
        green.setMode(DigitalChannel.Mode.OUTPUT);
        red = hardwareMap.get(DigitalChannel.class, "red");
        red.setMode(DigitalChannel.Mode.OUTPUT);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight = new Limelight();
        limelight.init(hardwareMap, imu, telemetry);

        limelight.setTeam(586);
    }

    @Override
    public void init_loop() {
        limelight.readObelisk();

        if (limelight.isDataCurrent) {
            green.setState(true);
            red.setState(false);
        } else {
            red.setState(true);
            green.setState(false);
        }
        telemetry.update();
    }

    public void start() {
        if (Blackboard.alliance == Blackboard.Alliance.RED) {
            limelight.setTeam(24);
        }
        else if (Blackboard.alliance == Blackboard.Alliance.BLUE) {
            limelight.setTeam(20);
        }
        collectorOn = true;
    }

    @Override
    public void loop() {
        targetInView = limelight.process();
        shooter.overridePower();

        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine();



        telemetry.addData("Left Joystick", "Drive");
        telemetry.addData("Right Joystick", "Rotate");
        telemetry.addLine();
        telemetry.addData("Left Bumper", "Very Slow Drive");
        telemetry.addData("Right Bumper", "Slow Drive");
        telemetry.addData("Right Trigger", "Shoot 3!x");
        telemetry.addData("Hold X", "Eject!");

        telemetry.addLine();
        telemetry.addLine("------------------------");
        telemetry.addLine();

        //Gamepad 1
        if (gamepad1.start) {
            imu.resetYaw();
        }
        handleShooting();
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
                    collectorOn = true;
                }
            }
        }
    }

    public boolean isWithinLeniencyRange() {
        return limelight.hasResults() && Math.abs(limelight.getTx()) <= aimLeniencyDegrees;
    }
}