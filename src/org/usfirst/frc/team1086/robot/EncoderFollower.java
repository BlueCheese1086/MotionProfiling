package org.usfirst.frc.team1086.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import jaci.pathfinder.Trajectory;

public class EncoderFollower {

    int encoder_offset, encoder_tick_count;
    double wheel_circumference;

    double kp, ki, kd, kv, ka;

    double last_error, heading;

    int segment;
    Trajectory trajectory;
    File file;
    BufferedWriter bw;
    public EncoderFollower(Trajectory traj) {
        this.trajectory = traj;
        file = new File("/home/lvuser/path.csv");
        try {
			bw = new BufferedWriter(new FileWriter(file));
			bw.write("Distance, Position, Velocity, Acceleration, Enc_Velocity, Output");
			bw.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.println("First catch");
			e.printStackTrace();
		}
    }

    public EncoderFollower() { }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     */
    public void setTrajectory(Trajectory traj) {
        this.trajectory = traj;
        reset();
    }

    /**
     * Configure the PID/VA Variables for the Follower
     * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
     * @param ki The integral term. Currently unused.
     * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
     * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
     *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
     *           motor controllers
     * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
     */
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = ka;
    }

    /**
     * Configure the Encoders being used in the follower.
     * @param initial_position      The initial 'offset' of your encoder. This should be set to the encoder value just
     *                              before you start to track
     * @param ticks_per_revolution  How many ticks per revolution the encoder has
     * @param wheel_diameter        The diameter of your wheels (or pulleys for track systems) in meters
     */
    public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter) {
        encoder_offset = initial_position;
        encoder_tick_count = ticks_per_revolution;
        wheel_circumference = Math.PI * wheel_diameter;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        last_error = 0; segment = 0;
    }

    /**
     * Calculate the desired output for the motors, based on the amount of ticks the encoder has gone through.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param encoder_tick The amount of ticks the encoder has currently measured.
     * @return             The desired output for your motor controller
     */
    public double calculate(int encoder_tick, double vel) {
        // Number of Revolutions * Wheel Circumference
        double distance_covered = ((double)(encoder_tick - encoder_offset) / encoder_tick_count)
                * wheel_circumference;
        //System.out.println("Distance covered: " + distance_covered);
        if (segment < trajectory.length()) {
            Trajectory.Segment seg = trajectory.get(segment);
            
            
            double error = seg.position - distance_covered;
            double calculated_value =
                    kp * error +                                    // Proportional
                    kd * ((error - last_error) / seg.dt) +          // Derivative
                    (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
            
            String line = distance_covered + ", " + seg.position + ", " + seg.velocity + ", " + seg.acceleration + ", " + vel + ", " + calculated_value +  "\n";
            last_error = error;
            heading = seg.heading;
            try {
				bw.write(line);
				bw.flush();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				System.out.println("Second catch");
				e.printStackTrace();
			}
            segment++;

            return calculated_value;
        } else {
        	try {
        		System.out.println("Closing file");
        		bw.flush();
				bw.close();
			} catch (IOException e) {
				System.out.println("Third catch");
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
        	return 0;
        }
        
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
        return trajectory.get(segment);
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {
        return segment >= trajectory.length();
    }

}