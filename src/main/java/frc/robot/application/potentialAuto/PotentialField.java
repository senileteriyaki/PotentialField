package frc.robot.application.potentialAuto

import edu.wpi.first.math.geometry.Translation2d;

public class PotentialField{ //represents a scalar potential field U, with force being -grad F
		
	public interface Obstacle {
		double getPotential(
			Translation2d position,
		);
	}

	public static class CircularObstacle implements Obstacle {
		private Translation2d center;
		private double strength; //so that negative strength is attractive
		private double actionDistance;
		private double radius;

		public CircularObstacle(Translation2d center, double strength, double actionDistance, double radius) {
			this.center = center;
			this.radius = radius;
			this.actionDistance = distance;
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

	public static class LinearObstacle implements Obstacle {
		private Translation2d start;
		private Translation2d end;
		private double strength; 
		private double actionDistance;

		public double getPotential(Translation2d position) {
			double dist = position
		}
	}

	private List<Obstacle> obstacles;
	private boolean multiplier;
	public PotentialField(double k) {
		obstacles = new ArrayList<Obstacle>;
		multiplier = k;
	}

	public void add(Obstacle o) {
		obstacles.add(o);
	}

	public double getTotalPotential(Translation2d position) {
		u = 0;
		for (obstacle : obstacles) {
			u += obstacle.getPotential(position);
		}
		return u;
	}
	
	public Translation2D calculateForce(Translation2d position, double h) {
		x = position.getX();
		y = position.getY(); 
		partialUx = (getTotalPotential(new Translation2d(x + h, y)) - getTotalPotential(new Translation2d(x - h, y))) * (1/(2*h));
		partialUy = (getTotalPotential(new Translation2d(x, y + h)) - getTotalPotential(new Translation2d(x, y - h))) * (1/(2*h));
		return new Translation2d(partialUx*multiplier, partialUy*multiplier);
	}
}


}
