package frc.robot.application.potentialAuto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.SwerveInput;

public class PotentialAuto extends SubsystemBase<PotentialAutoState>{
    private PotentialField field;
    private Pose2d[] forceArrows;
    private final Drive drive = Drive.getInstance();
    private Translation2d commandedFieldVelocity = Translation2d.kZero;
    private final double damping = 0.7;
    private final double maxAccel_ms2 = 1.0;
    private final double slowdownDistance_m = 1;
    private final double translation_kP = 2.5;
    private final double rotation_kP = 0.2;
    
    public PotentialAuto(PotentialField field) {
        super("PotentialAuto");
        this.field = field;
        setCommand(PotentialAutoState.IDLE);
    }

    public void inputPeriodic() {}

    public void outputPeriodic() {
        Logger.recordOutput("PotentialAuto/Force Arrows", forceArrows);

    }

    public void handle() {
        switch (getCommand()) {
            case DRIVE -> handleDrive();
            case FINISHED, DISABLED, IDLE -> {}
        }

    }

    public void handleDrive() {
        drive.setCommand(PathingMode.POSE_FOLLOWING);
        double step_size = 0.5;
        Pose2d robotPose = drive.getPose();
        Translation2d force = field.calculateForce(robotPose.getTranslation());
        force = force.div(force.getNorm());
        double distanceToGoal = robotPose.getTranslation().getDistance(field.getGoal());
        if (distanceToGoal < slowdownDistance_m) {
            step_size *= distanceToGoal / slowdownDistance_m;
        }
        Translation2d targetTranslation = robotPose.getTranslation().plus(force.times(step_size));
        Pose2d target = new Pose2d(targetTranslation, robotPose.getRotation());
        drive.setTargetPose(target, field.getMaxVelocity(), translation_kP, rotation_kP, true);
    }

    public void handleDriveOld() {
        drive.setCommand(PathingMode.FIELD_RELATIVE);
        ChassisSpeeds measuredRobot = drive.getChassisSpeeds();
        ChassisSpeeds measuredField = ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobot, drive.getRotation());
        Pose2d robotPose = drive.getPose();
        Translation2d acceleration = field.calculateForce(robotPose.getTranslation());
        acceleration = acceleration.minus(new Translation2d(measuredField.vxMetersPerSecond, measuredField.vyMetersPerSecond).times(damping));
        if (acceleration.getNorm() > maxAccel_ms2) {
            acceleration = acceleration.times(maxAccel_ms2 / acceleration.getNorm());
        }
        commandedFieldVelocity = commandedFieldVelocity.plus(acceleration.times(Constants.globalDelta_s));
        double vx = commandedFieldVelocity.getX();
        double vy = commandedFieldVelocity.getY();
        if (Constants.isRedAlliance()) {
            vx = -vx;
            vy = -vy;
        }
        SwerveInput input = new SwerveInput(vx/Drive.MAX_LINEAR_VEL_mps, vy/Drive.MAX_LINEAR_VEL_mps, 0, 0.8);
        drive.setInput(input);
        Logger.recordOutput("PotentialAuto/CommandedVelocity", commandedFieldVelocity);
        Logger.recordOutput("PotentialAuto/Acceleration", field.calculateForce(robotPose.getTranslation()));
    }


    public void start() {
        ChassisSpeeds measuredRobot = drive.getChassisSpeeds();
        ChassisSpeeds measuredField = ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobot, drive.getRotation());
        commandedFieldVelocity = new Translation2d(measuredField.vxMetersPerSecond, measuredField.vyMetersPerSecond);
        this.forceArrows = field.getForceArrows(Constants.FIELD_LENGTH_M, Constants.FIELD_WIDTH_M, 0.3, 0.01);
        drive.setCommand(PathingMode.POSE_FOLLOWING);
        setCommand(PotentialAutoState.DRIVE);

    }
}
