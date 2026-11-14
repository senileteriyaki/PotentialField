package frc.robot.application.potentialAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import frc.robot.util.Util;


public class PotentialField{ //represents a scalar potential field U, with force being -grad U
		
	public interface Potential {
		double getPotential(
			Translation2d position
		);
	}

	public static class GoalPotential implements Potential {
		private Translation2d goal;
		private double transitionDistance;
		private double strength; 

		public GoalPotential(Translation2d goal, double strength, double transitionDistance) {
			this.goal = goal;
			this.strength = strength;
			this.transitionDistance = transitionDistance;
		}

		public double getPotential(Translation2d position) {
			double dist = position.getDistance(goal);
			if (dist <= transitionDistance) {
				return 0.5 * strength * Math.pow(dist, 2);
			} else {
				return  strength * transitionDistance * (dist - transitionDistance/2);
			}

		}
	}

	public static class CircularObstacle implements Potential {
		private Translation2d center;
		private double strength; //so that negative strength is attractive
		private double actionDistance;
		private double radius;

		public CircularObstacle(Translation2d center, double strength, double actionDistance, double radius) {
			this.center = center;
			this.radius = radius;
			this.actionDistance = actionDistance;
			this.strength = strength;
		}

		public double getPotential(Translation2d position) {
			double dist = position.getDistance(center) - radius;
			if (dist >= actionDistance){
				return 0;
			}
			
			return strength * Math.pow(actionDistance - dist, 2);
		}
	}

	public static class LinearObstacle implements Potential {
		private Translation2d start;
		private Translation2d end;
		private double strength; 
		private double actionDistance;

		public LinearObstacle(Translation2d start, Translation2d end, double strength, double actionDistance) {
			this.start = start;
			this.end = end;
			this.strength = strength;
			this.actionDistance = actionDistance;
		}

		public double getPotential(Translation2d position) {
			double dist = Math.sqrt(Util.squaredDistanceToSegment(start, end, position));
			if (dist >= actionDistance) {
				return 0;
			}
			return strength * Math.pow(actionDistance - dist, 2);
		}
	}

	private List<Potential> potentials;
	private double multiplier;
	public PotentialField(double k) {
		potentials = new ArrayList<Potential>();
		multiplier = k;
	}

	public void add(Potential p) {
		potentials.add(p);
	}

	public double getTotalPotential(Translation2d position) {
		double u = 0;
		for (Potential potential : potentials) {
			u += potential.getPotential(position);
		}
		return u;
	}
	
	public Translation2d calculateForce(Translation2d position, double h) {
		double x = position.getX();
		double y = position.getY(); 
		double partialUx = (getTotalPotential(new Translation2d(x + h, y)) - getTotalPotential(new Translation2d(x - h, y))) * (1/(2*h));
		double partialUy = (getTotalPotential(new Translation2d(x, y + h)) - getTotalPotential(new Translation2d(x, y - h))) * (1/(2*h));
		return new Translation2d(-partialUx*multiplier, -partialUy*multiplier);
	}

	public Pose2d[] getForceArrows(double fieldLength, double fieldWidth, double spacing, double h) {
		List<Pose2d> arrows = new ArrayList<>();

		int nx = (int) Math.ceil(fieldLength / spacing);
		int ny = (int) Math.ceil(fieldWidth / spacing);

		for (int i = 0; i <= nx; i++) {
			for (int j = 0; j <= ny; j++) {

				double x = Math.min(i * spacing, fieldLength);
				double y = Math.min(j * spacing, fieldWidth);

				Translation2d position =
					new Translation2d(x, y);

				Translation2d force =
					calculateForce(position, h);

				if (force.getNorm() > 1e-6) {
					arrows.add(
						new Pose2d(position, force.getAngle())
					);
				}
			}
		}

		return arrows.toArray(new Pose2d[0]);
}
}

