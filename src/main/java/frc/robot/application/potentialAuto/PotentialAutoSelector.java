package frc.robot.application.potentialAuto;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.application.potentialAuto.PotentialField.CircularObstacle;
import frc.robot.application.potentialAuto.PotentialField.GoalPotential;
import frc.robot.application.potentialAuto.PotentialField.LineSegmentObstacle;
import frc.robot.application.potentialAuto.PotentialField.LinearObstacle;
import frc.robot.application.superstructure.SS.Flag;

/** Holds selectable autonomous routines. */
public class PotentialAutoSelector {
    private final List<PotentialActionList> routines = new ArrayList<>(); 
    private final List<PotentialField> fields = new ArrayList<>();
                                                                              
                                                                               
    private int index;

    public PotentialAutoSelector() {
        PotentialField testField = new PotentialField(1, new Translation2d(7, 7), 3);
        testField.add(new CircularObstacle(new Translation2d(4.5, 4), 1.5, 1, 2));
        testField.add(new LineSegmentObstacle(new Translation2d(3.5, 3.5), new Translation2d(2, 2), 2, 1));
        testField.add(new LinearObstacle(new Translation2d(8, 8), new Translation2d(8, 0), 3, 3));
        PotentialActionList testActions = new PotentialActionList(
                "Potential Auto",
                new Pose2d(1, 1, new Rotation2d()),
                PotentialActionList.PathingStyle.PRECISE)
                .startActions()

                .driveToPose(
                        new Pose2d(7, 7, new Rotation2d()),
                        testField.getMaxVelocity(),
                        2.5, 0.2,
                        15.0, 0.1)

                .waitFor(0.5, Flag.SCORE_HIGH)

                .driveToPose(
                        new Pose2d(3, 7, new Rotation2d()),
                        testField.getMaxVelocity(),
                        2.5, 0.2,
                        15.0, 0.2);

        routines.add(testActions);
        fields.add(testField);

    }

    public void add(PotentialActionList routine) {
        routines.add(routine);
    }

    public void nextAuto() {
        index = (index + 1) % routines.size();
    }

    public PotentialActionList getSelectedActionList() {
        return routines.get(index);
    }

    public PotentialField getSelectedField() {
        return fields.get(index);
    }

    public void periodic() {
        PotentialActionList selected = getSelectedActionList();
        Logger.recordOutput("Elastic/Selected Auto/Name", selected.getName());
        Logger.recordOutput("Elastic/Selected Auto/Index", index);
        // Logger.recordOutput("Elastic/Selected Auto/Start",
        selected.getStartDescription();
    }

}
