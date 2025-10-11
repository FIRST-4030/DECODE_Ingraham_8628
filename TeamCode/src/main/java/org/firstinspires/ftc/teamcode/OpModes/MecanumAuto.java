package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class MecanumAuto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime    = new ElapsedTime();
    private DcMotor     leftFront  = null;
    private DcMotor     rightFront = null;

    private int allianceSide;

    @Override
    public void init() {
        /*
         * Code that runs ONCE when the driver hits INIT
         *    Tasks to include are,
         *       - Initialize all motors, sensors, etc. via calls to "hardwareMap"
         *       - Initialize all class level variables
         */
        // Initialize the hardware variables
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        /*
         * Code that runs REPEATEDLY after the driver hits INIT, but before you hit START
         *    Tasks to include are,
         *       - Allow driver to set the alliance color
         */
        if(gamepad1.x){
            allianceSide = 20;
        }
        else if(gamepad1.b){
            allianceSide = 24;
        }
        telemetry.addData("Alliance Side:",allianceSide);
        telemetry.update();
    }

    @Override
    public void start() {
        /*
         * Code that runs ONCE when the driver hits START
         *    Tasks to include are,
         *       - Initialize April tags
         *       - Initialize any time parameters
         */
        runtime.reset();
    }

    @Override
    public void loop() {
        /*
         * Code that runs continuously throughout the entire autonomous period
         */
    }

    @Override
    public void stop() {
        /*
         * Code to run ONCE after the driver hits STOP
         *    Tasks to include are,
         *       - Stop motors
         *       - Close cameras
         */
    }
}
