package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterVelo {
    private DcMotorEx shooter;
    public double Kvelo = 0.0243; // power multiplier for rotations per second
    // FeedBack term is Kp (proportional term)
    // Set Kp to zero when tuning the Kvelo term!!
    public double Kp = 0.3;  // no gain in improvement when increasing beyond this

    static final double   COUNTS_PER_REV = 28 ;  // REV HD Hex 1:1 Motor Encoder

    public double targetVelocity = 0;  // rotations per second (max is ~40)

    public ShooterVelo(HardwareMap hardwareMap, String name, Boolean dir) {
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, name);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // WITH OUT!
        if (dir) {
            shooter.setDirection(DcMotor.Direction.FORWARD);
        } else {
            shooter.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public void overridePower() {
        double currentVelocity = shooter.getVelocity(AngleUnit.DEGREES)/COUNTS_PER_REV;
        double veloError = targetVelocity - currentVelocity;
        // CONTROLLER:  feedfoward = Kvelo + feedback = Kpos
        double setPower = targetVelocity * Kvelo  + veloError * Kp;
        shooter.setPower(setPower);
    }

    public void setControllerValues(double Kp, double Kvelo) {
        this.Kp = Kp;
        this.Kvelo = Kvelo;
    }

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
        if (0.9*targetVelocity < this.getVelocity() && this.getVelocity() < 1.1*targetVelocity) {
            return true;
        } else {
            return false;
        }
    }
}
