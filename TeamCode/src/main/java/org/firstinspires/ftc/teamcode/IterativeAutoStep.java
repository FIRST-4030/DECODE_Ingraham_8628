package org.firstinspires.ftc.teamcode;

import com.pedropathing.paths.PathChain;

public class IterativeAutoStep {
    public enum StepType {MOVE, SHOOT}


    private final StepType stepType;
    private final PathChain pathChain;
    private double startDelayMS = 0;
    private boolean collectorOn = false;
    private int targetShootCount = 3;


    private IterativeAutoStep(Builder builder) {
        this.stepType = builder.stepType;
        this.pathChain = builder.pathChain;
        this.startDelayMS = builder.startDelayMS;
        this.collectorOn = builder.collectorOn;
    }

    public StepType getStepType() {
        return stepType;
    }

    public PathChain getPathChain() {
        return pathChain;
    }

    public double getStartDelayMS() {
        return startDelayMS;
    }

    public boolean getCollectorOn() {
        return collectorOn;
    }

    public int getTargetShootCount() {
        return targetShootCount;
    }

    public static class Builder {
        private StepType stepType;
        private PathChain pathChain;
        private long startDelayMS = 0;
        private boolean collectorOn = false;
        private int targetShootCount = 3;

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

        public IterativeAutoStep build() {
            return new IterativeAutoStep(this);
        }
    }
}
