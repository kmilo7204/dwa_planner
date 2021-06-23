"""
DWA follows this algorithm:
    Calculate desired velocity (Transforming goal coordinates into velocities)
    Generate the linear velocity window and store the allowable linear velocities
    Generate the angular velocity window and store the allowable angular velocities
    Iterate trough each allowable linear velocity
    Iterate trough each allowable angular velocity 
    Calculate a trajectory based on the control inputs
    Calculate the distance to the close obstacle (Based on the ego position)
    Calculate the heading error between the current location and the goal
    Calculate the velocity difference between top speed and final trajectory velocity
    Maximize the cost function
    Returns the best trajectory
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.npyio import load
import robot_config

class DWA:
    # String literals 
    def __init__(self)-> None:
        self.config_params = robot_config.Parameters()
        pass

    # I should add the type of the incoming arguments 
    def update_motion(self, x: np.array, u: np.array)->list:
        """
        Update the robot motion based on the control commads (linear
        and angular velocity)

        Keyword arguments:
        x -- numpy array containing the current robot state -> [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        u -- numpy array containing the control commands 
        """
        # Yaw rate update
        x[2] += u[1] * self.config_params.dt

        # X and Y positions update
        x[0] += u[0] * math.cos(x[2]) * self.config_params.dt
        x[1] += u[0] * math.sin(x[2]) * self.config_params.dt
        
        # Linear and Angular velocities update
        x[3] = u[0]
        x[4] = u[1]

        # Return the updated state
        return x

    # I should add the type of the incoming arguments 
    def calculate_dw(self, x: np.array)->list:
        """
        Calculate the dynamic window, equivalent to the velocities that can be
        reached within the next time interval (dt). Accerelations are considered
        in this calculation as they are the hardware limitation.  
        
        Note that the dynamic window is centred around the actual velocity and 
        the extensions of it depend on the accelerations that can be exerted. All 
        curvatures outside the dynamic window cannot be reached within the next 
        time interval and thus are not considered for the obstacle avoidance.

        Keyword arguments:
        x -- numpy array containing the current robot state -> [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        """
        # We need to make sure we are within the motion constraints, in other words we need to 
        # lie within the dynamic window, for that reason we consider the velocity boundaries
        dw = [
            max(x[3] - self.config_params.max_a * self.config_params.dt, self.config_params.min_v), # Min lin. vel 
            min(x[3] + self.config_params.max_a * self.config_params.dt, self.config_params.max_v), # Max lin. vel
            max(x[4] - self.config_params.max_d_w * self.config_params.dt, self.config_params.min_w), # Min ang. vel
            min(x[4] + self.config_params.max_d_w * self.config_params.dt, self.config_params.max_w) # Max ang. vel
        ]

        # Dynamic window -> [V_min, V_max, W_min, W_max]  
        return dw

    def calculate_traj(self, x: np.array, v: float, w: float)->np.array:
        """
        Calculate a possible trajectory in a given time window. The calculation
        considers the currrent robot state and a single control input (As it assumes
        they are constants).  

        Keyword arguments:
        x -- numpy array containing the current robot state -> [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        v -- linear velocity for trajectory prediction
        w -- angular velocity for trajectory prediction  
        """
        # Temporal vector state
        x_tmp = np.array(x)
        # Array for saving the trajectory
        predicted_traj = np.array(x_tmp)
        # Control input
        u = np.array([v, w])
        time = 0.0

        # Prediction window (Lower than 3.0 seconds)
        while(time <= self.config_params.dw_time):
            # Update the motion and save the value for the trajectory
            x_tmp = self.update_motion(x_tmp, u)
            # Append the predicted state to the trajectory
            predicted_traj = np.vstack((predicted_traj, x_tmp))
            time += self.config_params.dt

        # Predicted trajectory array containing the trajectory as -> [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        # for each time step in the next three seconds.
        return predicted_traj
        
    def target_heading(self, trajectory: np.array, goal: np.array)->float:
        """
        Calcutes the heading cost between the current heading angle and the goal, 
        it is maximum when the robot is moving directly towards the target.

        Keyword arguments:
        trajectory -- numpy array containing the predicted trajectory
        """
        pos_x = trajectory[-1, 0]  # Extract the last X position in the trajectory 
        pos_y = trajectory[-1, 1]  # Extract the last Y position in the trajectory

        diff_x = goal[0] - pos_x
        diff_y = goal[1] - pos_y

        heading = math.atan2(diff_y, diff_x)

        # Calculate the difference or error between the trajectory angle and the the current yaw
        error = heading - trajectory[-1, 2]
        return abs(math.atan2(math.sin(error), math.cos(error)))

    def obstacle_distance(self, trajectory: np.array)->float:
        """
        Calculate the cost (Distance) to the closest obstacle in the road. When this 
        distance (Between the robot and obstacle) is small the robot will tend to move
        around it.

        Keyword arguments:
        trajectory -- numpy array containing the predicted trajectory
        """
        obst_x = self.config_params.obstacles[:, 0]
        obst_y = self.config_params.obstacles[:, 1]

        """
        Note:
        Keep in mind that the following operation returns an array for 
        each substraction. For example if there are 5 obstacles we will 
        calculate 5 different matrices between the X trajectory and the 
        5 obstacles positions in X, similarly this will take place in Y.

        traj[:, 0] = [1, 2, 3, 4, 5] -> Trajectory in X
        obs_x[:, None] = [1.5, 2.5, 3.1, 3.6, 4.0] -> Obstacles position in X
        traj[:, 1] - obs_x[:, None] = [
            [-0.5, 0.5, 1.5, 2.5, 3.5], -> X and first obstacle
            [-1.5, -0.5, 0.5, 1.5, 2.5], -> X and second obstacle
            [-2.1, -1.1, -0.1, 0.9, 1.9], -> X and third obstacle
            ...
            ]
        """
        # Calculate the difference between the trajectory and obstacles
        diff_x = trajectory[:, 0] - obst_x[:, None] # Broadcasting
        diff_y = trajectory[:, 1] - obst_y[:, None] # Broadcasting
        
        """
        Numpy hypot takes the distance of each point of the trajectory 
        to one given osbtacle and calculates the euclidean distance for 
        each individual point in the trajectory. Similarly we will have 
        the same number of vectors for each obstacle in the space. 
        
        Result will be an array contanining the euclidean distance from 
        each trajectory point to each of the obstacles, hence we will 
        have n array where n is given by the number of obstacles.
        """
        # Calculate the euclidean distance between trajectory and obstacles
        dist = np.array(np.hypot(diff_x, diff_y))

        # Some point in the whole array finds an obstacle
        if np.any(dist <= self.config_params.chassis_radius * 0.5):
            # Hit ane obstacle, hence discard this trajectory
            return float("inf")

        # We can either return the cost or the distance
        min_value = np.min(dist)

        # The closer we are the greater this number as the goal is to maximize
        # the cost function.
        return 1 / min_value

    def calculate_ctrl_traj(self, x: np.array, goal: np.array):
        """
        Calculates the optimal control input and trajectory from a given robot 
        state. This function performs the calculation of the dynamic window, the
        trajectory generation and the cost function calculation.

        It traverse all the possible trajectories and then return the optimal one.

        Keyword arguments:
        x -- numpy array containing the current robot state -> [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        """
        # Calculate the admissible velocities (DW) - [V_min, V_max, W_min, W_max] 
        dw_v = self.calculate_dw(x)

        # Array of velocities (Initial, Final, Resolution)
        v_vect = np.arange(dw_v[0], dw_v[1], self.config_params.v_res)
        w_vect = np.arange(dw_v[2], dw_v[3], self.config_params.w_res)

        minimum_cost = float("inf")

        optimal_u = [0.0, 0.0]
        optimal_traj = x

        # Iterate through the linear velocties
        for v in v_vect:
            # Iterate through the angular velocties
            for w in w_vect:
                # Trajectory generation for the following three seconds
                trajectory = self.calculate_traj(x, v, w)

                # Obstacle distance cost
                clearance_cost = self.config_params.distance_gain * self.obstacle_distance(trajectory) 
                # Heading angle cost
                heading_cost = self.config_params.heading_gain * self.target_heading(trajectory, goal)
    
                # Velocity cost. Difference between max velocity and final trajectory velocity
                velocity_cost = self.config_params.velocity_gain * abs(self.config_params.max_v - trajectory[-1, 3])

                total_cost = clearance_cost + heading_cost + velocity_cost

                if (minimum_cost >= total_cost):
                    # Set the new optimal cost
                    minimum_cost = total_cost
                    # Set the optimal control input
                    optimal_u = [v, w]
                    # Set the optimal trajectory
                    optimal_traj = trajectory
                    
                    # Both veloties are too small, we run into troublew if it happens
                    """
                    This is a particular scenario. It is not part of the algorithm, but
                    should be considered to find a solution. There are some scenarios
                    where the robot linear control velocity is too small as well as the 
                    current angular velocity and additionally it is moving toward an 
                    obstacle (Get stuck close to the obstacle), so we add some angular
                    velocity to force it to turn around the obstacle.
                    """
                    if (abs(optimal_u[0]) < 0.001 and abs(x[3]) < 0.001):
                        # We added some angular velocity so the robot turn around the obstacles  
                        # and calculates a new linear velocity
                        optimal_u[1] = -40.0 * math.pi / 180.0 
                        # At this point we can consider the heading angle and add or substract this
                        # angular velocity

        return optimal_u, optimal_traj

def main(x, goal):
    dynamic_window = DWA()
    trajectory = np.array(x)

    while(True):
        # Cleaning the plot
        plt.cla()

        # Calculate the optimal trajectory and control input
        u_control, pred_trajectory = dynamic_window.calculate_ctrl_traj(x, goal)
        # Update the robot state
        x = dynamic_window.update_motion(x, u_control)
        # Append the trajectory
        trajectory = np.vstack((trajectory, x))
        # Distance to goal
        dist_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])

        # Drawing the goal
        plt.plot(goal[0], goal[1], marker="o", markerfacecolor='blue')
        plt.plot(0.0, 0.0, marker="o", markerfacecolor='red')

        # Plot current trajectory
        plt.plot(pred_trajectory[:, 0], pred_trajectory[:, 1])
        # Plot obstacles
        plt.plot(dynamic_window.config_params.obstacles[:, 0], dynamic_window.config_params.obstacles[:, 1], "ok")
        # Draw robot position
        robot_body = plt.Circle((x[0] ,x[1]), 0.25, color='#00ffff')
        plt.gca().add_patch(robot_body)

        # Graph configuration
        plt.grid(True)        
        plt.axis("equal")
        plt.pause(0.001)

        if(dist_goal < 1.0):
            print("Path found")
            break

    plt.plot(trajectory[:, 0], trajectory[:, 1], '-r')
    plt.pause(0.001)
    plt.show()

if __name__ == "__main__":
    x_des = 10.0
    y_dex = 10.0

    # Set the initial state: [x, y, yaw, v, w]
    # x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    x = np.array([0.0, 0.0, math.pi, 0.0, 0.0])

    # Set goal location
    goal = np.array([x_des, y_dex])

    main(x, goal)


