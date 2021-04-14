# particle_filter_localization_project
## Team Members
Sophie Veys,
Josephine Passananti
## Plans
* *How you will initialize your particle cloud (initialize_particle_cloud)?*
    * We will create a 2D array to hold the particle locations and their weights. We will chose the particles with equal distribution across the map.
* *How you will update the position of the particles will be updated based on the movements of the robot(update_particles_with_motion_model)?*
    * When the robot moves we will move each particle the same distance and update their locations in the array.
* *How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?*
    * We will use the Monty Carlo algorithm with the sensor data from our robot to compute the new weights of each particle.
* *How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?*
    * First we will normalize the particles' weights by adding them and computing the proportion of each one to the total. We will then resample particles by keeping the particles with the most significant weights and cloning particles with respect to their proportion. We will do this in the draw_random_sample() function.
* *How you will update the estimated pose of the robot (update_estimated_robot_pose)?*
    * We will use our resampled particle distribution with the current estimated pose of the robot to update our belief of where the robot is.
* *How you will incorporate noise into your particle filter localization?*
    * We will shift particles slightly to add 'noise' to our environments and resample our particle cloud each time. We will add noise in each of the four directions. This will help us gain a better idea of where our robot is without having to move the robot itself.
## Timeline
We will work on initializing our particle cloud and updating our particles location and weights based on robot movement over the next few days and try to be done by the end of this weekend. We will work on resampling, updating robot location, and adding noise early next week. Hopefully we will have the remainder of the week to work on debugging and optimization.