import os

ISAACSIM_PYTHON = '/home/hajo/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh'
main_command = f'{ISAACSIM_PYTHON} examples/12_python_single_vehicle_gsl_benchmark.py'

start_id = 28
posittions = [[3.0, 3.0, 0.2],
              [7.5, 3.0, 0.2],
              [12.0,3.0, 0.2],
              [3.0, 7.5, 0.2],
              [7.5, 7.5, 0.2],
              [12.0,7.5, 0.2],
              [3.0, 12.0,0.2],
              [7.5, 12.0,0.2],
              [12.0,12.0,0.2]]

experiment_ids = [str(start_id + i).zfill(3) for i in range(len(posittions))]

def main():
    for _,(id,pos) in enumerate(zip(experiment_ids,posittions)):
        print(f"RUN EXPERIMENT: {id}  ---  POSITION: {pos}")
        os.system(f"{main_command} -- {id} '{pos}'")

if __name__ == "__main__":
    main()
