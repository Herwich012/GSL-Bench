import os
from datetime import datetime

def main():
    ISAACSIM_PYTHON = '/home/hajo/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh'
    main_command = f'{ISAACSIM_PYTHON} examples/12_python_single_vehicle_gsl_benchmark.py'

    start_id = 73 # starting experiment id
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

    start = datetime.now()

    for _,(id,pos) in enumerate(zip(experiment_ids,posittions)):
        exp_path = f"/home/hajo/0THESIS/experiments/{id}/"
        
        if not os.path.exists(exp_path): # create experiment folder if it does not exist yet
            os.system(f"mkdir -p {exp_path}") 
        
        print(f"RUN EXPERIMENT: {id}  ---  POSITION: {pos}")
        os.system(f"{main_command} -- {id} '{pos}'")

    print(f"Finished {len(experiment_ids)} experiments in {datetime.now() - start}")


if __name__ == "__main__":
    main()
