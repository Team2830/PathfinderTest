package org.usfirst.frc.team2830.robot.commands;

import java.io.File;

import org.usfirst.frc.team2830.robot.Robot;
import org.usfirst.frc.team2830.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */
public class FollowTrajectory extends Command {
	
	EncoderFollower left, right;
	File trajectoryPathFile;
    Notifier periodicRunnable;
	

    public FollowTrajectory() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this("/home/lvuser/myfile.csv");
    	
    }

    public FollowTrajectory(String pathFile) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
		File myFile = new File(pathFile);
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.resetCounters();
    	Robot.driveTrain.writeToSmartDashboard();
/*
 * 		We aren't using waypoints here to generate the path dynamically, but if we did, this is how you would do it.
		Waypoint[] points = new Waypoint[] {
				new Waypoint(0, 0, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
				new Waypoint(10, 0, 0)
		};
    	
<<<<<<< HEAD
		 Arguments:
		 Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
		 Sample Count:        SAMPLES_HIGH (100 000)
		                      SAMPLES_LOW  (10 000)
		                      SAMPLES_FAST (1 000)
		 Time Step:           0.02 Seconds
		 Max Velocity:        10.36 f/s
		 Max Acceleration:    2.0 f/s/s
		 Max Jerk:            60.0 f/s/s/s
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 5, 2.0, 60.0);
		Trajectory trajectory = Pathfinder.generate(points, config); */

		Trajectory trajectory = Pathfinder.readFromCSV(trajectoryPathFile);
=======
		// Arguments:
		// Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
		// Sample Count:        SAMPLES_HIGH (100 000)
//		                      SAMPLES_LOW  (10 000)
//		                      SAMPLES_FAST (1 000)
		// Time Step:           0.02 Seconds
		// Max Velocity:        10.36 f/s
		// Max Acceleration:    2.0 f/s/s
		// Max Jerk:            60.0 f/s/s/s
		//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 5, 2.0, 60.0);
		//Trajectory trajectory = Pathfinder.generate(points, config); 
		File myFile = new File("/home/lvuser/myfile.csv");
		Trajectory trajectory = Pathfinder.readFromCSV(myFile);
>>>>>>> refs/remotes/origin/master
		// Setting wheel base distance/
		TankModifier modifier = new TankModifier(trajectory).modify(2.05);
<<<<<<< HEAD
		try {
            left.reset();
            right.reset();
        } catch (Exception e) {
        }

=======
    	
>>>>>>> refs/remotes/origin/master
		left = new EncoderFollower(modifier.getLeftTrajectory());
    	right = new EncoderFollower(modifier.getRightTrajectory());
    	
<<<<<<< HEAD
/*    	 Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the 
    	 'getEncPosition' function.
    	 1000 is the amount of encoder ticks per full revolution
    	 Wheel Diameter is the diameter of your wheels (or pulley for a track system) in feet*/
=======
    	// Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the 
    	// 'getEncPosition' function.
    	// 1000 is the amount of encoder ticks per full revolution
    	// Wheel Diameter is the diameter of your wheels (or pulley for a track system) in meters
>>>>>>> refs/remotes/origin/master
    	left.configureEncoder(Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER), 4096, .5);
    	right.configureEncoder(Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER), 4096, .5);
    	
<<<<<<< HEAD
/*    	 The first argument is the proportional gain. Usually this will be quite high
    	 The second argument is the integral gain. This is unused for motion profiling
    	 The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
    	 The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
    	      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
    	 The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker*/
    	left.configurePIDVA(3.6, 0.0, 0.2, 1/10.36, .1);
    	right.configurePIDVA(3.6, 0.0, 0.2, 1/10.36, .1);
    	

        // Initialize the Notifier
        periodicRunnable = new Notifier(new PeriodicRunnable());

        // Start the notifier (notifier will be called every 0.02 second to ensure trajectory is read properly)
        periodicRunnable.startPeriodic(0.02);
=======
    	// The first argument is the proportional gain. Usually this will be quite high
    	// The second argument is the integral gain. This is unused for motion profiling
    	// The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
    	// The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
//    	      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
    	// The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
    	left.configurePIDVA(3.6, 0.0, 0.2, 1/10.36, .1);
    	right.configurePIDVA(3.6, 0.0, 0.2, 1/10.36, .1);
>>>>>>> refs/remotes/origin/master
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
<<<<<<< HEAD
=======
    	double l = left.calculate(Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER));
    	double r = right.calculate(Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER));


    	double gyro_heading = Robot.driveTrain.getAngle();    // Assuming the gyro is giving a value in degrees
    	double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

    	double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    	double turn = 0.8 * (-1.0/80.0) * angleDifference;

    	Robot.driveTrain.setLeft(l + turn);
    	Robot.driveTrain.setRight(r - turn);
>>>>>>> refs/remotes/origin/master
    	
<<<<<<< HEAD
=======
    	if (counter % 10 == 0 && !left.isFinished() && !right.isFinished()){
    		System.out.printf("Left Vel: %f\t", left.getSegment().velocity);
    		System.out.printf("Left Acc: %f\t", left.getSegment().acceleration);
    		System.out.printf("Left Pos: %f\t", left.getSegment().position);
    		
 //   		System.out.printf("Left Encoder: %d\t", Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER));
 //   		System.out.printf("Left Output: %f\t", l);
//    		System.out.printf("R-Y: %f\t", right.getSegment().y );
//    		System.out.printf("L-Y: %f\t", left.getSegment().y );
//    		System.out.printf("R-X: %f\t", right.getSegment().x );
//    		System.out.printf("L-X: %f\t", left.getSegment().x );

    		//	System.out.printf("Right Encoder: %d\t", Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER));
    	//	System.out.printf("Right Output: %f\t", r);
    	//	System.out.printf("Angle Diff: %f\t",angleDifference);
    	//	System.out.printf("angle correction: %f\n", turn);
    		System.out.println("end");
    	}
    	counter++;
    	Robot.driveTrain.writeToSmartDashboard();
>>>>>>> refs/remotes/origin/master
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(left.isFinished() && right.isFinished()){
        	System.out.println("finished reach called");
    		return true;

    	}return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stopDriving();
<<<<<<< HEAD
    	periodicRunnable.stop();
=======
>>>>>>> refs/remotes/origin/master
    	System.out.println("end called");
    	Robot.driveTrain.writeToSmartDashboard();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.stopDriving();
    	periodicRunnable.stop();    	
    }
    class PeriodicRunnable implements Runnable {
        int segment = 0;
    	int counter = 0;

        @Override
        public void run() {
        	double l = left.calculate(Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER));
        	double r = right.calculate(Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER));


        	double gyro_heading = Robot.driveTrain.getAngle();    // Assuming the gyro is giving a value in degrees
        	double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

        	double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        	double turn = 0.8 * (-1.0/80.0) * angleDifference;

        	Robot.driveTrain.setLeft(l + turn);
        	Robot.driveTrain.setRight(r - turn);
        	
        	if (counter % 10 == 0 && !left.isFinished() && !right.isFinished()){
        		System.out.printf("Left Vel: %f\t", left.getSegment().velocity);
        		System.out.printf("Left Acc: %f\t", left.getSegment().acceleration);
        		System.out.printf("Left Pos: %f\t", left.getSegment().position);
        		
     //   		System.out.printf("Left Encoder: %d\t", Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER));
     //   		System.out.printf("Left Output: %f\t", l);
//        		System.out.printf("R-Y: %f\t", right.getSegment().y );
//        		System.out.printf("L-Y: %f\t", left.getSegment().y );
//        		System.out.printf("R-X: %f\t", right.getSegment().x );
//        		System.out.printf("L-X: %f\t", left.getSegment().x );

        		//	System.out.printf("Right Encoder: %d\t", Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER));
        	//	System.out.printf("Right Output: %f\t", r);
        	//	System.out.printf("Angle Diff: %f\t",angleDifference);
        	//	System.out.printf("angle correction: %f\n", turn);
        		System.out.println("end");
        	}
        	counter++;
        	Robot.driveTrain.writeToSmartDashboard();
        }
    }
}
