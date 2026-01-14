package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    //    private DcMotorEx shooter;
    DcMotorEx shooter;
    Servo shooterHinge;

    public static double Kvelo = 0.0243; // power multiplier for rotations per second
    // FeedBack term is Kp (proportional term)
    // Set Kp to zero when tuning the Kvelo term!!
    public static double Kp = 0.3;  // no gain in improvement when increasing beyond this

    static final double   COUNTS_PER_REV = 28 ;  // REV HD Hex 1:1 Motor Encoder

    public double targetVelocity = 0;  // rotations per second (max is ~40)
    double range;

    public Shooter(HardwareMap hardwareMap, String name, Boolean dir) {
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, name);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // WITH OUT!
        setMotorDirection(dir);

        shooterHinge = hardwareMap.get(Servo.class, "shooterHinge");
        putHingeDown();
    }

    public void overridePower() {
        double currentVelocity = shooter.getVelocity(AngleUnit.DEGREES)/COUNTS_PER_REV;
        double veloError = targetVelocity - currentVelocity;
        // CONTROLLER:  feedfoward = Kvelo + feedback = Kpos
        double setPower = targetVelocity * Kvelo  + veloError * Kp;
        shooter.setPower(setPower);
    }

    private void setMotorDirection(Boolean dir) {
        //True = forward, false = backwards
        if (dir) {
            shooter.setDirection(DcMotor.Direction.FORWARD);
        } else {
            shooter.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void setControllerValues(double Kp, double Kvelo) {
        this.Kp = Kp;
        this.Kvelo = Kvelo;
    }

    public void setPower(double power) {
        shooter.setPower(power);
    }

//    public static void fireVolleySorted(Limelight limelight, Telemetry telemetry, Servo flipper, Shooter shooterLeft, Servo launchFlapLeft, Shooter shooterRight, Servo launchFlapRight, LinearOpMode opMode) {
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        double shooterVelo = 0;
//        while (timer.seconds() < 0.1) {
//            limelight.process(telemetry);
//            shooterVelo = shooterLeft.getShooterVelo(limelight);
//        }
//        if (limelight.getObelisk().equals("PGP")) {
//            fireShooter(shooterVelo, shooterLeft, launchFlapLeft);
//            flipper.setPosition(1);
//            opMode.sleep(500);
//            fireShooter(shooterVelo, shooterLeft, launchFlapLeft);
//            flipper.setPosition(0.525);
//        } else if (limelight.getObelisk().equals("GPP")) {
//            fireShooter(shooterVelo, shooterLeft, launchFlapLeft);
//            flipper.setPosition(1);
//            opMode.sleep(500);
//            fireShooter(shooterVelo, shooterLeft, launchFlapLeft);
//            flipper.setPosition(0.525);
//        } else if (limelight.getObelisk().equals("PPG")) {
//            fireShooter(shooterVelo, shooterLeft, launchFlapLeft);
//            flipper.setPosition(1);
//            opMode.sleep(500);
//            fireShooter(shooterVelo, shooterLeft, launchFlapLeft);
//            flipper.setPosition(0.525);
//        }
//    }

    public void shoot(double velo) {
        targetVelocity = velo;
        ElapsedTime shooterTimer = new ElapsedTime();

        while (!atSpeed()) {
            overridePower();
        }

        shooterTimer.reset();
        shooterHinge.setPosition(0.55);

        while (shooterTimer.milliseconds() < 500) {
            overridePower();
        }

        shooterHinge.setPosition(0.25);

        while (shooterTimer.milliseconds() < 1000) {
            overridePower();
        }
    }

    public void stopShooter() {
        targetVelocity = 0;
    }

    public void putHingeUp() {
        shooterHinge.setPosition(0.55);
    }

    public void putHingeDown() {
        shooterHinge.setPosition(0.25);
    }

    public double getShooterVelo(Limelight limelight) {
        // compute velocity from range using function based on shooting experiments
        range = limelight.getRange();
        if (range < 80) {
            double poly = 29;
            return poly;
        } else {
            double poly = 19 + 0.125 * range;
            return poly;
        }
    }
//    public static void fireShooter(double velocity, Shooter shooterRight, Servo launchFlapRight) {
//        shooterRight.targetVelocity = velocity;
//        ElapsedTime timer = new ElapsedTime();
//
//        while (!shooterRight.atSpeed()) {
//            shooterRight.overridePower();
//        }
//        timer.reset();
//        launchFlapRight.setPosition(Constants.rightFlapUp);
//        while (timer.seconds() < 0.6) {
//            shooterRight.overridePower();
//        }
//        launchFlapRight.setPosition(Constants.rightFlapDown);
//        while (timer.seconds() < 1) {
//            shooterRight.overridePower();
//        }
//    }

    public void setTargetVelocity(double velo) {
        this.targetVelocity = velo;
    }

    public double getPower() {
        return shooter.getPower();
    }

    public double getVelocity() {
        return shooter.getVelocity(AngleUnit.DEGREES)/COUNTS_PER_REV;
    }

    public boolean atSpeed() {
        if (0.98*targetVelocity < this.getVelocity() && this.getVelocity() < 1.02*targetVelocity) {
            return true;
        } else {
            return false;
        }
    }
}
