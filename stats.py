
from TaskInspectionPlanner import TaskInspectionPlanner
from MapEnvironment import MapEnvironment
import json
from matplotlib import pyplot as plt
import numpy as np

def create_planner(json_file):
    return TaskInspectionPlanner(MapEnvironment(json_file), coverage=1)


def get_data(planner, outfile):

    for i in range(10):
        print(f"run {i+1}")
        planner.reset()
        planner.plan()  

    stats = {}

    for i, stat in enumerate(planner.stats):
        stats[str(i)] = stat


    with open(outfile, "w") as outfile:
        json.dump(stats, outfile)

    

def get_stats(json_file):

    with open(json_file, 'r') as file:
        stats = json.load(file)
    
    # print(stats)
    cost, time, coverage = [], [], []
    for _,v in stats.items():
        cost.append(v[0])
        time.append(v[1])
        coverage.append(v[2])

    for i in range(len(cost)):
        plt.plot(time[i], coverage[i])
    plt.legend([f'plot {i+1}' for i in range(len(cost))])
    plt.xlabel('time')
    plt.ylabel('coverage')
    plt.grid()
    plt.show()

    for i in range(len(cost)):
        plt.plot(coverage[i], cost[i])
    plt.legend([f'plot {i+1}' for i in range(len(cost))])
    plt.xlabel('coverage')
    plt.ylabel('path length')
    plt.grid()
    plt.show()


if __name__=='__main__':
    planner = create_planner('map_plan_p2.json')
    get_data(planner, 'stats_E2_20_shmeeejson')
    # get_stats('stats_E2_20_beam.json')
    

    


    