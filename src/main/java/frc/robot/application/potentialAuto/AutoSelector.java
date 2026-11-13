package frc.robot.application.potentialAuto;


import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.application.potentialAuto.PotentialField.CircularObstacle;
import frc.robot.application.potentialAuto.PotentialField.GoalPotential;

/** Holds selectable autonomous routines. */
public class AutoSelector {
    private final ArrayList<PotentialField> routines = new ArrayList<>(); // this should really be a separate class with a potential field + superstructure actions
    private int index;

    public AutoSelector() {
        PotentialField testField = new PotentialField(1);
        testField.add(new CircularObstacle(new Translation2d(1, 1), 1, 0.5, 0.2));
        testField.add(new GoalPotential(new Translation2d(5, 5), 1, 1));
        routines.add(testField);
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
