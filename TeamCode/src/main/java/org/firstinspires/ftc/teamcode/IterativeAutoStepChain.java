package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IterativeAutoStepChain {
    public IterativeAutoStep[] iterativeAutoSteps;
    public double collectorSpeed;
    public boolean done = false;


    public int activeStepIndex = 0;
    private ElapsedTime currentWaitTime = new ElapsedTime();


    private int currentShootCount = 0;
    private ElapsedTime currentShotTime = new ElapsedTime();
    private final double SHOOTER_HINGE_LIFT_DURATION_MS = 400;
    private final double SHOT_DURATION_MS = 800;


    private boolean finishedWaiting = false;
    private boolean shooterReachedSpeed = false;


    public IterativeAutoStepChain(double collectorSpeedValue, IterativeAutoStep[] iterativeAutoStepsValue) {
        collectorSpeed = collectorSpeedValue;
        iterativeAutoSteps = iterativeAutoStepsValue;
    }

    public void update(Follower follower, DcMotorEx collector, Shooter shooter, Limelight limelight, Telemetry telemetry, Chassis chassis) {
        if (done) {
            telemetry.addLine("Done");
            return;
        }

        IterativeAutoStep activeIterativeAutoStep = iterativeAutoSteps[activeStepIndex];

        // This condition passes only the FIRST frame we are finished waiting
        if (currentWaitTime.milliseconds() >= activeIterativeAutoStep.getStartDelayMS() && !finishedWaiting) {
            finishedWaiting = true;

            if (activeIterativeAutoStep.getStepType() == IterativeAutoStep.StepType.MOVE) {
                follower.followPath(activeIterativeAutoStep.getPathChain());
            }
        }

        telemetry.addData("Start Delay", activeIterativeAutoStep.getStartDelayMS());
        telemetry.addData("Current wait time", currentWaitTime.milliseconds());
        telemetry.addData("Step Index", activeStepIndex);
        telemetry.addData("Follower busy?", follower.isBusy());

        follower.update();

        if (!finishedWaiting) { return; }

        if (activeIterativeAutoStep.getCollectorOn()) {
            collector.setPower(collectorSpeed);
        } else if (activeIterativeAutoStep.getStepType() == IterativeAutoStep.StepType.SHOOT) {
            collector.setPower(-collectorSpeed);
        } else {
            collector.setPower(0);
        }

        telemetry.addLine("--- STEP-TYPE-SPECIFIC EXECUTION ---");
        telemetry.addData("StepType", activeIterativeAutoStep.getStepType());

        switch (activeIterativeAutoStep.getStepType()) {
            case MOVE:
                if (!follower.isBusy()) {
                    nextStep(follower, collector, shooter);
                }
                break;
            case SHOOT:
                if (limelight.isDataCurrent) {
                    chassis.turnTo(limelight.getTx(), 0);
                }
                telemetry.addData("Shooter target velocity", shooter.targetVelocity);
                int targetShootCount = activeIterativeAutoStep.getTargetShootCount();

                if (currentShootCount < targetShootCount) {
                    shooter.setTargetVelocity(shooter.getShooterVelo(limelight));
                    shooter.overridePower();

                    telemetry.addData("Current shot time", currentShotTime.milliseconds());

                    if (shooter.atSpeed()) {
                        shooterReachedSpeed = true;
                    }

                    if (!shooterReachedSpeed) {
                        currentShotTime.reset();
                        return;
                    }

                    if (currentShotTime.milliseconds() < SHOOTER_HINGE_LIFT_DURATION_MS) {
                        shooter.putHingeDown();
                        telemetry.addLine("KEEP IT DOWN!");
                    } else if (currentShotTime.milliseconds() < SHOT_DURATION_MS) {
                        shooter.putHingeUp();
                        telemetry.addLine("SHOOTTT!!!");
                    } else {
                        currentShootCount ++;
                        currentShotTime.reset();
                    }
                }

                if (currentShootCount == targetShootCount) {
                    nextStep(follower, collector, shooter);
                }
                break;
        }
    }

    public void init() {
        activeStepIndex = 0;
        done = false;

        currentShootCount = 0;
        currentWaitTime.reset();
        currentShotTime.reset();
        finishedWaiting = false;
        shooterReachedSpeed = false;

        IterativeAutoStep activeIterativeAutoStep = iterativeAutoSteps[activeStepIndex];
    }

    public void nextStep(Follower follower, DcMotorEx collector, Shooter shooter) {
        currentShootCount = 0;
        currentWaitTime.reset();
        currentShotTime.reset();
        finishedWaiting = false;
        shooterReachedSpeed = false;

        shooter.setTargetVelocity(0);
        shooter.putHingeDown();

        int lastActiveStepIndex = activeStepIndex;

        if (activeStepIndex >= iterativeAutoSteps.length - 1) {
            done = true;
            collector.setPower(0);
            shooter.stopShooter();
            return;
        } else {
            activeStepIndex ++;
        }

        IterativeAutoStep lastIterativeAutoStep = iterativeAutoSteps[lastActiveStepIndex];
        IterativeAutoStep activeIterativeAutoStep = iterativeAutoSteps[activeStepIndex];

        follower.setMaxPower(activeIterativeAutoStep.getMaxPower());
    }
}
