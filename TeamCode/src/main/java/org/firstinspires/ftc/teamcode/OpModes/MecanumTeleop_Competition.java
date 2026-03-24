package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Common_Teleop;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.Shooter;

@Disabled
@Configurable
@TeleOp(name = "Mecanum Teleop Limelight", group = "Robot")
public class MecanumTeleop_Competition extends OpMode {

    //Will replace MecanumTeleop Limelight

    public static double collectorSpeed = 0.45;

    public static int polyRangeCrossover = 80;
    public static int polyVeloBaseFar = 19;
    public static int polyVeloBaseNear = 29;
    public static double polyVeloBaseRangeFactor = 0.125;
    public static double aimLeniencyDegrees = 3;

    Chassis chassis;
    DcMotorEx collector;
    Shooter shooter;
    Servo liftServo;
    Limelight limelight;
    IMU imu;
    Common_Teleop common;

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
        common = new Common_Teleop(telemetry, hardwareMap, gamepad1, gamepad2);
        common.init();
    }

    @Override
    public void init_loop() {
       common.init_loop();
    }

    public void start() {
        common.start();
    }

    @Override
    public void loop() {
        common.loop();
    }
}