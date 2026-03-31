package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common_Teleop;
@Disabled
@Configurable
@TeleOp(name = "Mecanum Teleop Competition", group = "Robot")
public class MecanumTeleop_Competition extends OpMode {
//This opmode can replace mecanum teleop limelight
    public static double collectorSpeed = 0.45;

    Common_Teleop common;

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