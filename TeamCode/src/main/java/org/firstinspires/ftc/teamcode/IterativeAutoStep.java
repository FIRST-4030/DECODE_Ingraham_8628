package org.firstinspires.ftc.teamcode;

import com.pedropathing.paths.PathChain;

public class IterativeAutoStep {
    public enum StepType {MOVE, SHOOT}


    private final StepType stepType;
    private final PathChain pathChain;
    private final double startDelayMS;
    private final boolean collectorOn;
    private final int targetShootCount;
    private final float maxPower;


    private IterativeAutoStep(Builder builder) {
        this.stepType = builder.stepType;
        this.pathChain = builder.pathChain;
        this.startDelayMS = builder.startDelayMS;
        this.collectorOn = builder.collectorOn;
        this.targetShootCount = builder.targetShootCount;
        this.maxPower = builder.maxPower;
    }

    public StepType getStepType() { return stepType; }

    public PathChain getPathChain() { return pathChain; }

    public double getStartDelayMS() { return startDelayMS; }

    public boolean getCollectorOn() { return collectorOn; }

    public int getTargetShootCount() { return targetShootCount; }

    public float getMaxPower() { return maxPower; }

    public static class Builder {
        private StepType stepType;
        private PathChain pathChain;
        private long startDelayMS = 0;
        private boolean collectorOn = false;
        private int targetShootCount = 0;
        private float maxPower = 1;

        public Builder setStepType(StepType value) {
            stepType = value;
            return this;
        }

        public Builder setPathChain(PathChain value) {
            pathChain = value;
            return this;
        }

        public Builder setStartDelayMS(long value) {
            startDelayMS = value;
            return this;
        }

        public Builder setCollectorOn(boolean value) {
            collectorOn = value;
            return this;
        }

        public Builder setTargetShootCount(int value) {
            targetShootCount = value;
            return this;
        }

        public Builder setMaxPower(float value) {
            maxPower = value;
            return this;
        }

        public IterativeAutoStep build() {
            return new IterativeAutoStep(this);
        }
    }
}
