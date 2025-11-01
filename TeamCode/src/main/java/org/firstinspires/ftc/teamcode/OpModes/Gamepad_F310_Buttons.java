package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name="Gamepad F310 Buttons")
public class Gamepad_F310_Buttons extends OpMode
{

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {}

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop() {
        telemetry.addLine(String.format("A:  %b,        B:  %b", gamepad1.a,gamepad1.b));
        telemetry.addLine(String.format("X:  %b,        Y:  %b",gamepad1.x,gamepad1.y));
        telemetry.addLine("");

        telemetry.addLine(String.format("Bumper - Left:  %b,        Right:  %b", gamepad1.left_bumper,gamepad1.right_bumper));
        telemetry.addLine("");

        telemetry.addLine(String.format("Trigger -  Left:  %6.4f,    Right:  %6.4f", gamepad1.left_trigger,gamepad1.right_trigger));
        telemetry.addLine("");

        telemetry.addLine(String.format("DPAD  - Up:      %b,       Down:  %b", gamepad1.dpad_up,gamepad1.dpad_down));
        telemetry.addLine(String.format("               Left:    %b,       Right:   %b", gamepad1.dpad_left,gamepad1.dpad_right));
        telemetry.addLine("");

        telemetry.addLine(String.format("Left Stick X :    %6.4f", gamepad1.left_stick_x));
        telemetry.addLine(String.format("Left Stick Y:     %6.4f", gamepad1.left_stick_y));
        telemetry.addLine(String.format("Right Stick X:   %6.4f", gamepad1.right_stick_x));
        telemetry.addLine(String.format("Right Stick Y:   %6.4f", gamepad1.right_stick_y));

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

}
