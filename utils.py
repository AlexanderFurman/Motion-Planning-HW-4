import numpy as np

def compute_inner_interpolated_configurations(config1, config2):
    '''
    A function two interpolate between two configurations and return the set of configurations in between, or nothing if they are too close.
    @param config1 The source configuration of the robot.
    @param config2 The destination configuration of the robot.
    '''

    # compute number of steps
    required_diff = 0.01
    interpolation_steps = int(np.linalg.norm(config2 - config1)//required_diff) - 1

    # return interpolated configurations
    if interpolation_steps > 0:
        return np.linspace(start=config1, stop=config2, num=interpolation_steps+2), interpolation_steps
    else:
        return None, None

def write_plan_stats(path_coverage, path_cost, computation_time):
    '''
    Write plan stats to a file.
    @param path_coverage The coverage of the plan.
    @param path_cost The cost of the plan (in C-space).
    @param computation_time The time it took to compute the plan.
    '''
    # interpolate plan
    file_lines = [f"Coverage: {path_coverage}\n",
                  f"Cost: {path_cost}\n",
                  f"Computation Time: {computation_time}\n"]

    # write plan to file
    f = open("plan_stats.txt", "w")
    f.writelines(file_lines)
    f.close()