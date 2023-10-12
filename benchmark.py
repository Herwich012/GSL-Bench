import os
from pathlib import Path
from datetime import datetime
HOME_DIR = Path.home()
PEGASUS_DIR = f"{HOME_DIR}/Omniverse_extensions/PegasusSimulator"
# TODO: make benchmark class with automatic experiment tree genereation, ids, logging etc.

def multiple_exp(main_commands, start_ids, env_ids): # multiple experiments
    for env_id in env_ids:
        for command,start_id in zip(main_commands,start_ids):
            grid_exp(command, start_id, env_id)


def grid_exp(command, id, env): # grid experiment
    start_id = id # starting experiment id
    posittions = [[3.0, 3.0, 0.2]]
                #   [7.5, 3.0, 0.2],
                #   [12.0,3.0, 0.2],
                #   [3.0, 7.5, 0.2],
                #   [7.5, 7.5, 0.2],
                #   [12.0,7.5, 0.2],
                #   [3.0, 12.0,0.2],
                #   [7.5, 12.0,0.2],
                #   [12.0,12.0,0.2]]

    experiment_ids = [str(start_id + i).zfill(3) for i in range(len(posittions))]

    start = datetime.now()

    for _,(id,pos) in enumerate(zip(experiment_ids,posittions)):
        exp_path = f"{PEGASUS_DIR}/examples/results/{id}/"
        
        if not os.path.exists(exp_path): # create experiment folder if it does not exist yet
            os.system(f"mkdir -p {exp_path}") 
        
        print(f"RUN EXPERIMENT: {id} --- ENV: {env} --- POSITION: {pos}")
        os.system(f"{command} -- {id} {env} '{pos}'")

    print(f"Finished {len(experiment_ids)} experiments in {datetime.now() - start}")


if __name__ == "__main__":
    ISAACSIM_PYTHON = f'{HOME_DIR}/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh'
    main_commands = [f'{ISAACSIM_PYTHON} examples/12_python_single_vehicle_gsl_benchmark.py']
    start_ids = [199]
    env_ids = [1,2,3,4,5,6]

    # grid_exp(main_commands[0],start_ids[0], env_ids[0])
    multiple_exp(main_commands,start_ids, env_ids)
