package frc.robot.application.potentialAuto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.application.superstructure.SS;
import frc.robot.application.superstructure.SS.Flag;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;

/** Runs one PotentialActionList, using a persistent PotentialField for navigation. */
public class PotentialAuto extends SubsystemBase<PotentialAutoState> {
    private static final double LOOKAHEAD_m = 0.5;
    private static final double SLOWDOWN_DISTANCE_m = 1.0;
    private static final double COMPLETION_HEADING_TOLERANCE_rad = Math.toRadians(5.0);
    private static final double NO_COMPLETION_START_s = -1.0;
    private static final double FORCE_EPSILON = 1e-6;
    private static final double FORCE_ARROW_SPACING_m = 0.3;
    private static final double FORCE_DIFFERENCE_STEP_m = 0.01;
    private static final double FULL_TURN_deg = 180.0;

    private final PotentialField field;
    private final PotentialActionList actions;
    private final Drive drive = Drive.getInstance();
    private final SS superstructure = SS.getInstance();

    private Pose2d[] forceArrows = new Pose2d[0];
    private double completionStart_s = NO_COMPLETION_START_s;
    private boolean waiting = false;
    private boolean targetsFlipped = false;

    public PotentialAuto(PotentialField field, PotentialActionList actions) {
        super("PotentialAuto");
        if (field == null || actions == null) {
            throw new IllegalArgumentException("PotentialField and PotentialActionList must not be null");
        }
        this.field = field;
        this.actions = actions;
        refreshForceArrows();
        setCommand(PotentialAutoState.IDLE);
    }

    @Override
    public void inputPeriodic() {}

    @Override
    protected void handle() {
        if (DriverStation.isDisabled()) {
            if (getCommand() != PotentialAutoState.DISABLED) stop();
            return;
        }

        switch (getCommand()) {
            case DRIVE -> {
                if (waiting) {
                    handleWait();
                } else {
                    handleDrive();
                }
            }
            case FINISHED, DISABLED, IDLE -> { }
        }
    }

    /** Starts the action list, optionally resetting odometry to its blue-alliance start pose. */
    public void start() {
        actions.reset();
        waiting = false;
        completionStart_s = NO_COMPLETION_START_s;

        // The action goals are authored in blue coordinates. Toggle the 180-degree field
        // transform only when the alliance state differs from the last start, avoiding
        // accidentally transforming the same targets twice on repeated runs.
        boolean redAlliance = Constants.isRedAlliance();
        if (redAlliance != targetsFlipped) {
            actions.transformTargets(pose -> new Pose2d(
                    Constants.FIELD_LENGTH_M - pose.getX(),
                    Constants.FIELD_WIDTH_M - pose.getY(),
                    pose.getRotation().rotateBy(Rotation2d.fromDegrees(FULL_TURN_deg))));
            targetsFlipped = redAlliance;
        }

        resetStartingPose();
        startAction();
    }

    /** Stops the routine and clears all superstructure requests. */
    public void stop() {
        waiting = false;
        clearSuperstructureFlags();
        drive.setCommand(PathingMode.DISABLED);
        setCommand(PotentialAutoState.DISABLED);
    }

    public boolean isFinished() {
        return getCommand() == PotentialAutoState.FINISHED;
    }

    private void resetStartingPose() {
        Pose2d blueStartPose = actions.startPose();
        if (blueStartPose == null) {
            return;
        }

        // Drive.resetGyroAndPose() applies the alliance heading convention itself, so mirror
        // only x/y here and pass through the blue-authored rotation.
        Pose2d startPose = Constants.isRedAlliance()
                ? new Pose2d(
                        Constants.FIELD_LENGTH_M - blueStartPose.getX(),
                        Constants.FIELD_WIDTH_M - blueStartPose.getY(),
                        blueStartPose.getRotation())
                : blueStartPose;

        drive.resetGyroAndPose(startPose.getRotation().getDegrees(), startPose);
        Logger.recordOutput("PotentialAuto/StartPose", startPose);
    }

    /** Initializes the current action and changes only the PotentialField goal for DRIVE actions. */
    private void startAction() {
        PotentialActionList.Action action = actions.currentAction();
        completionStart_s = NO_COMPLETION_START_s;
        commandTimer.reset();

        if (action == null) {
            waiting = false;
            clearSuperstructureFlags();
            drive.setCommand(PathingMode.DISABLED);
            setCommand(PotentialAutoState.FINISHED);
            return;
        }

        applySuperstructureFlags(action);

        if (action.type == PotentialActionList.ActionType.DRIVE) {
            waiting = false;
            field.changeGoal(action.getTargetPose().getTranslation());
            refreshForceArrows();
            setCommand(PotentialAutoState.DRIVE);

            // Reset follower state once at the beginning of a PRECISE action. Intermediate
            // targets move every loop, so subsequent updates always preserve follower state.
            commandPotentialTarget(action, action.pathingStyle == PotentialActionList.PathingStyle.CONTINUOUS);
        } else {
            waiting = true;
            drive.setCommand(PathingMode.DISABLED);
            setCommand(PotentialAutoState.DRIVE);
        }

        Logger.recordOutput("PotentialAuto/ActionIndex", actions.getIndex());
        Logger.recordOutput("PotentialAuto/ActionType", action.type.toString());
    }

    private void handleWait() {
        PotentialActionList.Action action = actions.currentAction();
        if (action == null || commandTimer.time() >= action.timeout_s) {
            advance();
        }
    }

    private void handleDrive() {
        PotentialActionList.Action action = actions.currentAction();
        if (action == null) {
            advance();
            return;
        }

        if (commandTimer.time() >= action.timeout_s) {
            Logger.recordOutput("PotentialAuto/ActionTimedOut", true);
            advance();
            return;
        }

        Pose2d currentPose = drive.getPose();
        if (isAtTarget(action, currentPose)) {
            // Keep commanding the final pose while any requested post-arrival dwell elapses.
            drive.setTargetPose(
                    action.getTargetPose(),
                    effectiveMaxVelocity(action),
                    action.translation_kP,
                    action.rotation_kP,
                    true);

            if (completionStart_s < 0.0) {
                completionStart_s = commandTimer.time();
            } else if (commandTimer.time() - completionStart_s >= action.minimumDwell_s) {
                advance();
            }
        } else {
            completionStart_s = NO_COMPLETION_START_s;
            commandPotentialTarget(action, true);
        }
    }

    /** Updates the potential-field goal's local lookahead pose and sends it to Drive's pose follower. */
    private void commandPotentialTarget(PotentialActionList.Action action, boolean preserveMomentum) {
        Pose2d currentPose = drive.getPose();
        Translation2d currentPosition = currentPose.getTranslation();
        Translation2d force = field.calculateForce(currentPosition);
        double forceMagnitude = force.getNorm();

        Translation2d intermediatePosition = currentPosition;
        if (forceMagnitude > FORCE_EPSILON) {
            Translation2d direction = force.div(forceMagnitude);
            double distanceToGoal = currentPosition.getDistance(action.getTargetPose().getTranslation());
            double stepSize = LOOKAHEAD_m * Math.min(1.0, distanceToGoal / SLOWDOWN_DISTANCE_m);
            intermediatePosition = currentPosition.plus(direction.times(stepSize));
        }
        // in case force is too small we continue 
        Pose2d intermediateTarget = new Pose2d(
                intermediatePosition,
                action.getTargetPose().getRotation());

        drive.setTargetPose(
                intermediateTarget,
                effectiveMaxVelocity(action),
                action.translation_kP,
                action.rotation_kP,
                preserveMomentum);

        Logger.recordOutput("PotentialAuto/Force", force);
        Logger.recordOutput("PotentialAuto/ForceMagnitude", forceMagnitude);
        Logger.recordOutput("PotentialAuto/IntermediateTarget", intermediateTarget);
        Logger.recordOutput("PotentialAuto/Goal", action.getTargetPose());
    }

    private double effectiveMaxVelocity(PotentialActionList.Action action) {
        return Math.min(action.maxVelocity_mps, field.getMaxVelocity());
    }

    private boolean isAtTarget(PotentialActionList.Action action, Pose2d currentPose) {
        Pose2d target = action.getTargetPose();
        double translationError = currentPose.getTranslation().getDistance(target.getTranslation());
        double headingError = Math.abs(
                target.getRotation().minus(currentPose.getRotation()).getRadians());
        return translationError <= action.positionTolerance_m
                && headingError <= COMPLETION_HEADING_TOLERANCE_rad;
    }

    private void advance() {
        actions.pass();
        startAction();
    }

    private void applySuperstructureFlags(PotentialActionList.Action action) {
        for (Flag flag : Flag.values()) {
            superstructure.set(flag, action.flags.contains(flag));
        }
    }

    private void clearSuperstructureFlags() {
        for (Flag flag : Flag.values()) {
            superstructure.disable(flag);
        }
    }

    private void refreshForceArrows() {
        forceArrows = field.getForceArrows(
                Constants.FIELD_LENGTH_M,
                Constants.FIELD_WIDTH_M,
                FORCE_ARROW_SPACING_m,
                FORCE_DIFFERENCE_STEP_m);
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("PotentialAuto/Name", actions.getName());
        Logger.recordOutput("PotentialAuto/ActionIndex", actions.getIndex());
        Logger.recordOutput("PotentialAuto/AllianceRed", Constants.isRedAlliance());
        Logger.recordOutput("PotentialAuto/Goal", field.getGoal());
        Logger.recordOutput("PotentialAuto/ForceArrows", forceArrows);
        Logger.recordOutput("PotentialAuto/Waiting", waiting);
    }
}
