package frc.robot.application.potentialAuto;


import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.application.potentialAuto.PotentialField.CircularObstacle;
import frc.robot.application.potentialAuto.PotentialField.GoalPotential;
import frc.robot.application.potentialAuto.PotentialField.LineSegmentObstacle;
import frc.robot.application.potentialAuto.PotentialField.LinearObstacle;

/** Holds selectable autonomous routines. */
public class PotentialAutoSelector {
    private final ArrayList<PotentialField> routines = new ArrayList<>(); // this should really be a separate class with a potential field + superstructure actions
    private int index;

    public PotentialAutoSelector() {
        PotentialField testField = new PotentialField(1, new Translation2d(7, 7), 3);
        testField.add(new GoalPotential(new Translation2d(7, 7), 1, 3));
        testField.add(new CircularObstacle(new Translation2d(4.5, 4), 1.5, 1,2));
        testField.add(new LineSegmentObstacle(new Translation2d(3.5, 3.5), new Translation2d(2, 2), 2, 1));
        testField.add(new LinearObstacle(new Translation2d(8, 8), new Translation2d(8, 0), 3, 3));
        routines.add(testField);

        PotentialField simpleField = new PotentialField(1, new Translation2d(3, 4), 2);
        simpleField.add(new GoalPotential(new Translation2d(3, 4), 1, 3));
        routines.add(simpleField);
    }

    public void add(PotentialField routine) { routines.add(routine); }
    public void nextAuto() { index = (index + 1) % routines.size(); }
    public PotentialField getSelected() { return routines.get(index); }

    public void periodic() {
        PotentialField selected = getSelected();
        //Logger.recordOutput("Elastic/Selected Auto/Name", selected.getName());
        Logger.recordOutput("Elastic/Selected Auto/Index", index);
        //Logger.recordOutput("Elastic/Selected Auto/Start", selected.getStartDescription());
    }
    
}
