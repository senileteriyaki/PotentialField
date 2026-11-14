package frc.robot.application.potentialAuto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class PotentialAuto extends SubsystemBase<PotentialAutoState>{
    private PotentialField field;
    private Pose2d[] forceArrows;
    private final Drive drive = Drive.getInstance();
    
    public PotentialAuto(PotentialField field) {
        super("PotentialAuto");
        this.field = field;
        this.forceArrows = field.getForceArrows(Constants.FIELD_LENGTH_M, Constants.FIELD_WIDTH_M, 0.3, 0.01);
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
        
    }


    public void start() {

    }
}
