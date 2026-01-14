package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Limelight_anglesetter;

@TeleOp(name = "Limelight Angle Setter")
public class LimelightAngleSetter extends OpMode {

    Limelight_anglesetter limelight;
    IMU imu;

    double cameraAngle = 0.032;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight_anglesetter.class, "limelight");

    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            limelight.setTeam(20);
        }
        else if (gamepad1.bWasPressed()) {
            limelight.setTeam(24);
        }

        telemetry.addData("Team", limelight.getTeam());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("ADJUST CAMERA ANGLE USING DPAD LEFT AND RIGHT");
        telemetry.addLine("TO GET THE CORRECT COMPUTED RANGE");

        if (gamepad1.dpadLeftWasPressed()) {
            limelight.setCameraAngle(cameraAngle += 0.002);
        } else if (gamepad1.dpadRightWasPressed()) {
            limelight.setCameraAngle(cameraAngle -= 0.002);
        }

        telemetry.addData("Camera Angle ", cameraAngle);
        telemetry.update();

        limelight.process(telemetry);
    }
}
