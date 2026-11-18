package frc.robot.application.potentialAuto;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.application.superstructure.SS.Flag;

/**
 * Declarative sequence of potential-field drive actions and timed superstructure actions.
 *
 * Drive targets are authored in blue-alliance field coordinates. PotentialAuto mirrors the
 * targets for red alliance when the routine starts. The PotentialField's obstacle collection
 * is not owned or modified by this class; each DRIVE action only supplies a new goal pose.
 */
public class PotentialActionList {
    private static final double DEFAULT_DRIVE_TIMEOUT_s = 15.0;
    private static final double DEFAULT_POSITION_TOLERANCE_m = 0.08;
    private static final double DEFAULT_MINIMUM_DWELL_s = 0.0;
    private static final double START_ACTION_DELAY_s = 0.34;

    public enum ActionType {
        DRIVE,
        WAIT
    }

    /**
     * PRECISE resets the pose follower at the beginning of a new drive action.
     * CONTINUOUS preserves its motion state when moving between drive actions.
     * PotentialAuto preserves follower state between intermediate-goal updates in either mode.
     */
    public enum PathingStyle {
        PRECISE,
        CONTINUOUS
    }

    public static class Action {
        public final ActionType type;
        private Pose2d targetPose;
        public final double maxVelocity_mps;
        public final double translation_kP;
        public final double rotation_kP;
        public final double timeout_s;
        public final double minimumDwell_s;
        public final double positionTolerance_m;
        public final EnumSet<Flag> flags;
        public final PathingStyle pathingStyle;

        private Action(
                ActionType type,
                Pose2d targetPose,
                double maxVelocity_mps,
                double translation_kP,
                double rotation_kP,
                double timeout_s,
                double minimumDwell_s,
                double positionTolerance_m,
                EnumSet<Flag> flags,
                PathingStyle pathingStyle) {
            this.type = type;
            this.targetPose = targetPose;
            this.maxVelocity_mps = maxVelocity_mps;
            this.translation_kP = translation_kP;
            this.rotation_kP = rotation_kP;
            this.timeout_s = timeout_s;
            this.minimumDwell_s = minimumDwell_s;
            this.positionTolerance_m = positionTolerance_m;
            this.flags = EnumSet.copyOf(flags);
            this.pathingStyle = pathingStyle;
        }

        public Pose2d getTargetPose() {
            return targetPose;
        }
    }

    private final String name;
    /** Blue-alliance starting pose. Null means do not reset odometry in PotentialAuto.start(). */
    private final Pose2d blueStartPose;
    private final PathingStyle pathingStyle;
    private final ArrayList<Action> actions = new ArrayList<>();
    private int index = 0;

    /** Creates a routine that does not reset odometry when it starts. */
    public PotentialActionList(String name) {
        this(name, null, PathingStyle.PRECISE);
    }

    /** Creates a routine with an explicit blue-alliance starting pose. */
    public PotentialActionList(String name, Pose2d blueStartPose) {
        this(name, blueStartPose, PathingStyle.PRECISE);
    }

    public PotentialActionList(String name, Pose2d blueStartPose, PathingStyle pathingStyle) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("Action list name must not be blank");
        }
        if (pathingStyle == null) {
            throw new IllegalArgumentException("Pathing style must not be null");
        }
        this.name = name;
        this.blueStartPose = blueStartPose;
        this.pathingStyle = pathingStyle;
    }

    /** Adds a drive-to-pose action with the default timeout and no post-arrival dwell. */
    public PotentialActionList driveToPose(
            Pose2d targetPose,
            double maxVelocity_mps,
            double translation_kP,
            double rotation_kP,
            Flag... flags) {
        return driveToPose(
                targetPose,
                maxVelocity_mps,
                translation_kP,
                rotation_kP,
                DEFAULT_DRIVE_TIMEOUT_s,
                DEFAULT_MINIMUM_DWELL_s,
                DEFAULT_POSITION_TOLERANCE_m,
                flags);
    }

    /**
     * Adds a drive-to-pose action. The shared PotentialField's goal is changed to this pose's
     * translation when the action begins; the field's obstacle list remains unchanged.
     */
    public PotentialActionList driveToPose(
            Pose2d targetPose,
            double maxVelocity_mps,
            double translation_kP,
            double rotation_kP,
            double timeout_s,
            double minimumDwell_s,
            Flag... flags) {
        return driveToPose(
                targetPose,
                maxVelocity_mps,
                translation_kP,
                rotation_kP,
                timeout_s,
                minimumDwell_s,
                DEFAULT_POSITION_TOLERANCE_m,
                flags);
    }

    /** Adds a drive action with explicit timeout, dwell, and position tolerance. */
    public PotentialActionList driveToPose(
            Pose2d targetPose,
            double maxVelocity_mps,
            double translation_kP,
            double rotation_kP,
            double timeout_s,
            double minimumDwell_s,
            double positionTolerance_m,
            Flag... flags) {
        if (targetPose == null) {
            throw new IllegalArgumentException("Drive target pose must not be null");
        }
        if (!(maxVelocity_mps > 0.0)
                || !(timeout_s > 0.0)
                || minimumDwell_s < 0.0
                || !(positionTolerance_m > 0.0)) {
            throw new IllegalArgumentException("Invalid drive-action parameters");
        }

        actions.add(new Action(
                ActionType.DRIVE,
                targetPose,
                maxVelocity_mps,
                translation_kP,
                rotation_kP,
                timeout_s,
                minimumDwell_s,
                positionTolerance_m,
                enumSet(flags),
                pathingStyle));
        return this;
    }

    /** Adds a timed superstructure action. The drivetrain is disabled during this action. */
    public PotentialActionList waitFor(double timeout_s, Flag... flags) {
        if (timeout_s < 0.0) {
            throw new IllegalArgumentException("Wait timeout must be nonnegative");
        }
        actions.add(new Action(
                ActionType.WAIT,
                null,
                0.0,
                0.0,
                0.0,
                timeout_s,
                0.0,
                DEFAULT_POSITION_TOLERANCE_m,
                enumSet(flags),
                pathingStyle));
        return this;
    }

    /** Adds the same short initial superstructure delay used by the regular ActionList. */
    public PotentialActionList startActions() {
        return waitFor(START_ACTION_DELAY_s);
    }

    private static EnumSet<Flag> enumSet(Flag... flags) {
        EnumSet<Flag> result = EnumSet.noneOf(Flag.class);
        if (flags != null) {
            for (Flag flag : flags) {
                if (flag != null) result.add(flag);
            }
        }
        return result;
    }

    public void reset() {
        index = 0;
    }

    /** Transforms drive goals (not the field obstacles or the blue starting pose). */
    public void transformTargets(Function<Pose2d, Pose2d> transform) {
        for (Action action : actions) {
            if (action.targetPose != null) {
                action.targetPose = transform.apply(action.targetPose);
            }
        }
    }

    public Action currentAction() {
        return index < actions.size() ? actions.get(index) : null;
    }

    public void pass() {
        index++;
    }

    public int getIndex() {
        return index;
    }

    public int size() {
        return actions.size();
    }

    public String getName() {
        return name;
    }

    public Pose2d startPose() {
        return blueStartPose;
    }

    public String getStartDescription() {
        return blueStartPose == null ? "CURRENT_POSE" : "CUSTOM";
    }
}
