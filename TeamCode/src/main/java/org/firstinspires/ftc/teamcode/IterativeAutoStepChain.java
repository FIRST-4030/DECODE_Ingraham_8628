package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IterativeAutoStepChain {
    public IterativeAutoStep[] iterativeAutoSteps;
    public double collectorSpeed;
    public double shootingVelocity;
    public boolean done = false;


    public int activeStepIndex = 0;
    private ElapsedTime currentWaitTime = new ElapsedTime();


    private int currentShootCount = 0;
    private ElapsedTime currentShotTime = new ElapsedTime();
    private boolean finishedWaiting = false;


    public IterativeAutoStepChain(double collectorSpeedValue, IterativeAutoStep[] iterativeAutoStepsValue) {
        iterativeAutoSteps = iterativeAutoStepsValue;
        collectorSpeed = collectorSpeedValue;
    }

    public void update(Follower follower, DcMotorEx collector, Telemetry telemetry) {
        telemetry.update();

        if (done) {
            telemetry.addLine("Done");
            return;
        }

        IterativeAutoStep activeIterativeAutoStep = iterativeAutoSteps[activeStepIndex];

        // This condition passes only the FIRST frame we are finished waiting
        if (currentWaitTime.milliseconds() >= activeIterativeAutoStep.getStartDelayMS() && !finishedWaiting) {
            finishedWaiting = true;
            follower.followPath(activeIterativeAutoStep.getPathChain());
        }

        telemetry.addData("Start Delay", activeIterativeAutoStep.getStartDelayMS());
        telemetry.addData("Current wait time", currentWaitTime.milliseconds());
        telemetry.addData("Step Index", activeStepIndex);
        telemetry.addData("Follower busy?", follower.isBusy());

        follower.update();

        if (!finishedWaiting) { return; }

        if (activeIterativeAutoStep.getCollectorOn()) {
            collector.setPower(collectorSpeed);
        } else {
            collector.setPower(0);
        }

        switch (activeIterativeAutoStep.getStepType()) {
            case MOVE:
                if (!follower.isBusy()) {
                    nextStep(follower, collector);
                }
            case SHOOT: // WIP
                int targetShootCount = activeIterativeAutoStep.getTargetShootCount();

                if (currentShootCount <= targetShootCount) {

                }

                // We've finished shooting if we're one ABOVE shoot count this frame
                if (currentShootCount > targetShootCount) {
                    nextStep(follower, collector);
                }
        }
    }

    public void init(Follower follower) {
        activeStepIndex = 0;
        done = false;
        currentShootCount = 0;
        currentWaitTime.reset();
        currentShotTime.reset();

        IterativeAutoStep activeIterativeAutoStep = iterativeAutoSteps[activeStepIndex];
    }

    public void nextStep(Follower follower, DcMotorEx collector) {
        currentWaitTime.reset();
        currentShotTime.reset();
        currentShootCount = 0;
        finishedWaiting = false;

        int lastActiveStepIndex = activeStepIndex;

        if (activeStepIndex >= iterativeAutoSteps.length - 1) {
            done = true;
            collector.setPower(0);
            return;
        } else {
            activeStepIndex ++;
        }

        IterativeAutoStep lastIterativeAutoStep = iterativeAutoSteps[lastActiveStepIndex];
        IterativeAutoStep activeIterativeAutoStep = iterativeAutoSteps[activeStepIndex];
    }
}
