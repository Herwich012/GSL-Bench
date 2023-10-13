import os
from pathlib import Path
from datetime import datetime
from typing import Any
HOME_DIR = Path.home()
PEGASUS_DIR = f"{HOME_DIR}/Omniverse_extensions/PegasusSimulator"
CURR_DIR = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve()) # Get current directory
ISAACSIM_PYTHON = f'{HOME_DIR}/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh'
DEFAULT_POS = [[3.0, 3.0, 0.2]
               [7.5, 3.0, 0.2],
               [12.0,3.0, 0.2],
               [3.0, 7.5, 0.2],
               [7.5, 7.5, 0.2],
               [12.0,7.5, 0.2],
               [3.0, 12.0,0.2],
               [7.5, 12.0,0.2],
               [12.0,12.0,0.2]]
# TODO: make benchmark class with automatic experiment tree genereation, ids, logging etc.


class Benchmark:
    def __init__(self, benchmark:dict={}) -> None:
        self.start_time =   datetime.now()
        self.scripts =      benchmark.get('scripts', ['examples/12_python_single_vehicle_gsl_benchmark.py'])
        self.envs =         benchmark.get('envs', [1])
        self.exp_start_id = benchmark.get('start_id', None)
        self.pos_list =     benchmark.get('pos_list', DEFAULT_POS)
        self.pos_select =   benchmark.get('pos_select', [0])
        self.comment =      benchmark.get('comment', '')

        if self.exp_start_id == None:
            self.get_start_id()

        self.benchmark_list = self.get_benchmark_list()

    
    def get_start_id(self) -> None:
        pass


    def get_benchmark_list(self) -> list:
        benchmark_list = []
        exp_id = self.exp_start_id
        
        for _,script in enumerate(self.scripts):
            for _,env in enumerate(self.envs):
                for _,pos in enumerate(self.pos_list):
                    benchmark_list.append([str(exp_id).zfill(3), script, env, pos])
                    exp_id += 1

        return benchmark_list


    def save_benchmark_list(self) -> None:
        pass


    def execute(self, parameters) -> None:
        command = f'{ISAACSIM_PYTHON} {parameters[1]}'
        exp_id = str(parameters[0])
        env_id = str(parameters[2])
        pos =    parameters[3]

        print(f"RUN EXPERIMENT: {exp_id} --- ENV: {env_id} --- POSITION: {pos}")
        os.system(f"{command} -- {exp_id} {env_id} '{pos}'")    


    def prompt(self) -> bool:
        print("The following experiments will be run:")
        print("exp_id                      script                         env_id     pos")
        print("--------------------------------------------------------------------------------")
        print(*self.benchmark_list, sep = '\n')
        #     ['001', 'examples/12_python_single_vehicle_gsl_benchmark.py', 5, [7.5, 12.0,0.2]]
        answer = input("Continue? y/n:")
        
        if answer == 'y' or answer == 'Y':
            return True
        else:
            return False


    def run(self):
        if self.prompt():
            self.save_benchmark_list()

            for benchmark in self.benchmark_list:
                self.execute(benchmark)
        
        else:
            print("Benchmark canceled")
            return


    def is_float(self, element:any) -> bool:
        #If you expect None to be passed:
        if element is None: 
            return False
        try:
            float(element)
            return True
        except ValueError:
            return False


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
    main_commands = [f'{ISAACSIM_PYTHON} examples/12_python_single_vehicle_gsl_benchmark.py']
    start_ids = [199]
    env_ids = [1,2,3,4,5,6]

    # grid_exp(main_commands[0],start_ids[0], env_ids[0])
    multiple_exp(main_commands,start_ids, env_ids)
