package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester", group = "FTC")
public class ServoTester extends OpMode {

    String DEVICE_NAME = "shooterHinge";

    Servo   servo;
    double  position = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, DEVICE_NAME);
    }

    @SuppressLint("DefaultLocale")
    public void init_loop() {
        if (gamepad1.dpadUpWasReleased()) {
            position += 0.1;
        }

        if (gamepad1.dpadDownWasReleased()) {
            position -= 0.1;
        }

        telemetry.addLine(String.format("Servo starts at %.1f", position));
        telemetry.addLine("DPad Up: Increases servo start position");
        telemetry.addLine("DPad Down: Decreases servo start position");
        telemetry.update();
    }

    public void start() {
        servo.setPosition(position);
    }

    public void loop() {
        telemetry.addLine("DPad Left: Move servo down");
        telemetry.addLine("DPad Right: Move servo up");
        telemetry.addData("Servo Position", "%.1f", position);
        telemetry.update();

        if (gamepad1.dpadLeftWasReleased()) {
            position -= 0.1;
            position = Math.max(0., position);
            servo.setPosition(position);
        }

        if (gamepad1.dpadRightWasReleased()) {
            position += 0.1;
            position = Math.min(1., position);
            servo.setPosition(position);
        }
    }
}