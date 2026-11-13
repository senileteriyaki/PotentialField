package frc.robot.application.potentialAuto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemBase;

public class PotentialAuto extends SubsystemBase<PotentialAutoState>{
    private PotentialField field;
    private Pose2d[] forceArrows;
    
    public PotentialAuto(PotentialField field) {
        super("PotentialAuto");
        this.field = field;
        this.forceArrows = field.getForceArrows(Constants.FIELD_LENGTH_M, Constants.FIELD_WIDTH_M, 0.3, 0.01);
    }

    public void inputPeriodic() {

    }

    public void outputPeriodic() {
        Logger.recordOutput("Force Arrows", forceArrows);
    }

    public void handle() {

    }
}
