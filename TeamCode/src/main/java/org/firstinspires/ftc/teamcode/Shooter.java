package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Shooter {
    DcMotorEx shooter;
    double power=0;

    public void init(HardwareMap hardwareMap ){
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getVelocity(){
        return shooter.getVelocity();
    }

    public void initPower(double power){
        this.power = power;
    }

    public void shoot(){
        shooter.setPower(power);
    }

    public void stop(){
        shooter.setPower(0);
    }

    public double adjustPower(double value){
        power += value;
        return power;
    }
//shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

}
