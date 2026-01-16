package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Limelight;

@Disabled
@TeleOp(name = "Limelight Angle Setter")
public class LimelightAngleSetter extends OpMode {

    Limelight limelight;

    IMU imu;

    double cameraAngle;

    @Override
    public void init() {

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

        cameraAngle = limelight.getCameraAngle();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Set alliance: X = Blue, B = Red");
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
        telemetry.addLine("Adjust camera angle using DPAD Left and Right");
        telemetry.addLine("to get the correct computed range");

        if (gamepad1.dpadLeftWasPressed()) {
            limelight.setCameraAngle(cameraAngle += 0.001);
        } else if (gamepad1.dpadRightWasPressed()) {
            limelight.setCameraAngle(cameraAngle -= 0.001);
        }

        telemetry.addData("Camera Angle ", cameraAngle);
        telemetry.addData("limelight Range", limelight.getRange());
        telemetry.update();

        limelight.process();
    }
}