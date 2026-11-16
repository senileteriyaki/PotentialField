package frc.robot.application.potentialAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import frc.robot.util.Util;

public class PotentialField { // represents a scalar potential field U, with force being -grad U

	public interface Potential {
		double getPotential(
				Translation2d position,
				Translation2d goal);
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

		public double getPotential(Translation2d position, Translation2d goal) {
			double dist = position.getDistance(goal);
			if (dist <= transitionDistance) {
				return 0.5 * strength * Math.pow(dist, 2);
			} else {
				return strength * transitionDistance * (dist - transitionDistance / 2);
			}

		}
	}

	public static class CircularObstacle implements Potential {
		private Translation2d center;
		private double strength; // so that negative strength is attractive
		private double actionDistance;
		private double radius;

		public CircularObstacle(Translation2d center, double radius, double actionDistance, double strength) {
			this.center = center;
			this.radius = radius;
			this.actionDistance = actionDistance;
			this.strength = strength;
		}

		public double getPotential(Translation2d position, Translation2d goal) {
			double dist = position.getDistance(center) - radius;
			if (dist >= actionDistance) {
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

		public double getPotential(Translation2d position, Translation2d goal) {
			double dist = Math.sqrt(Util.squaredDistanceToSegment(start, end, position));
			if (dist >= actionDistance) {
				return 0;
			}
			return strength * Math.pow(actionDistance - dist, 2);
		}
	}

	public static class TeardropObstacle implements Potential {

		private final Translation2d location;

		private final double primaryStrength;
		private final double primaryMaxRange;
		private final double primaryRadius;

		private final double tailStrength;
		private final double tailLength;

		public TeardropObstacle(
				Translation2d location,
				double primaryStrength,
				double primaryMaxRange,
				double primaryRadius,
				double tailStrength,
				double tailLength) {
			if (primaryRadius < 0
					|| primaryMaxRange <= primaryRadius
					|| tailLength <= 0
					|| primaryStrength < 0
					|| tailStrength < 0) {
				throw new IllegalArgumentException(
						"Invalid teardrop obstacle parameters");
			}

			this.location = location;
			this.primaryStrength = primaryStrength;
			this.primaryMaxRange = primaryMaxRange;
			this.primaryRadius = primaryRadius;
			this.tailStrength = tailStrength;

			// Additional tail length, following the original
			// implementation's parameter convention.
			this.tailLength = primaryMaxRange + tailLength;
		}

		@Override
		public double getPotential(
				Translation2d position,
				Translation2d goal) {
			double potential = getRadialPotential(position);
			// Direction from goal through the reef.
			Translation2d awayFromGoal = location.minus(goal);
			double goalDistance = awayFromGoal.getNorm();
			// No meaningful tail orientation if the goal
			// coincides with the reef.
			if (goalDistance < 1e-9) {
				return potential;
			}
			Translation2d ex = awayFromGoal.div(goalDistance);
			Translation2d ey = new Translation2d(-ex.getY(), ex.getX());
			// Coordinates relative to the reef.
			Translation2d relative = position.minus(location);
			double x = relative.getX() * ex.getX()
					+ relative.getY() * ex.getY();
			double y = relative.getX() * ey.getX()
					+ relative.getY() * ey.getY();
			// Tail only exists behind the reef.
			if (x <= 0 || x >= tailLength) {
				return potential;
			}
			double s = x / tailLength;
			// Width tapers to zero at the tail endpoint.
			double halfWidth = primaryMaxRange * (1.0 - s * s);
			if (halfWidth < 1e-9) {
				return potential;
			}
			double q = y / halfWidth;
			// Outside the tail's lateral influence.
			if (Math.abs(q) >= 1.0) {
				return potential;
			}
			// Tail strength: rises smoothly to maximum
			// at primaryMaxRange, then smoothly decreases.
			double envelope;
			if (x <= primaryMaxRange) {
				envelope = smoothstep(x / primaryMaxRange);
			} else {
				envelope = 1.0 - smoothstep(
						(x - primaryMaxRange)
								/ (tailLength - primaryMaxRange));
			}
			// Smooth lateral potential profile.
			double lateralProfile = 1.0 - q * q;
			lateralProfile *= lateralProfile;
			double tailPotential = tailStrength * envelope * lateralProfile;
			return potential + tailPotential;
		}

		private double getRadialPotential(
				Translation2d position) {
			double surfaceDistance = position.getDistance(location) - primaryRadius;

			double influenceDistance = primaryMaxRange - primaryRadius;

			double penetration = Math.max(
					0.0,
					influenceDistance - surfaceDistance);

			return 0.5 * primaryStrength
					* penetration * penetration;
		}

		private static double smoothstep(double t) {
			t = Math.max(0.0, Math.min(1.0, t));

			return t * t * (3.0 - 2.0 * t);
		}
	}

	private List<Potential> potentials;
	private double multiplier;
	private Translation2d goal;

	public PotentialField(double k, Translation2d goal) {
		potentials = new ArrayList<Potential>();
		multiplier = k;
		this.goal = goal;
	}

	public void add(Potential p) {
		potentials.add(p);
	}

	public double getTotalPotential(Translation2d position) {
		double u = 0;
		for (Potential potential : potentials) {
			u += potential.getPotential(position, goal);
		}
		return u;
	}

	public Translation2d calculateForce(Translation2d position, double h) {
		double x = position.getX();
		double y = position.getY();
		double partialUx = (getTotalPotential(new Translation2d(x + h, y))
				- getTotalPotential(new Translation2d(x - h, y))) * (1 / (2 * h));
		double partialUy = (getTotalPotential(new Translation2d(x, y + h))
				- getTotalPotential(new Translation2d(x, y - h))) * (1 / (2 * h));
		return new Translation2d(-partialUx * multiplier, -partialUy * multiplier);
	}

	public Translation2d calculateForce(Translation2d position) {
		return calculateForce(position, 0.01);
	}

	public Pose2d[] getForceArrows(double fieldLength, double fieldWidth, double spacing, double h) {
		List<Pose2d> arrows = new ArrayList<>();

		int nx = (int) Math.ceil(fieldLength / spacing);
		int ny = (int) Math.ceil(fieldWidth / spacing);

		for (int i = 0; i <= nx; i++) {
			for (int j = 0; j <= ny; j++) {

				double x = Math.min(i * spacing, fieldLength);
				double y = Math.min(j * spacing, fieldWidth);

				Translation2d position = new Translation2d(x, y);

				Translation2d force = calculateForce(position, h);

				if (force.getNorm() > 1e-6) {
					arrows.add(
							new Pose2d(position, force.getAngle()));
				}
			}
		}

		return arrows.toArray(new Pose2d[0]);
	}
}
