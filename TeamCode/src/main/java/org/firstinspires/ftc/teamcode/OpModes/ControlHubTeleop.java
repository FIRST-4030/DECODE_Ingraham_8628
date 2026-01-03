package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.ControlHub;

@TeleOp(name="ControlHubTeleop")
public class ControlHubTeleop extends OpMode {

    ControlHub controlHub;
    boolean finished = false;

    @Override
    public void init() {
        controlHub = new ControlHub();
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (finished) { return; }

        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
        if (!controlHub.isMacAddressValid()) {
            controlHub.reportBadMacAddress(telemetry, hardwareMap);
            finished = true;
        } else {
            telemetry.addData("MAC Address:", controlHub.getMacAddress());
            telemetry.addData("Network Name:", controlHub.getNetworkName());
            telemetry.addData("Comment:", controlHub.getComment());
            telemetry.update();
        }
    }
}