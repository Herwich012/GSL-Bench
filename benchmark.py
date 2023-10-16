import os
import sys
import numpy as np
from pathlib import Path
from datetime import datetime

argv = sys.argv
try:
    Y_ARG = argv[1]
except IndexError:
    Y_ARG = None

HOME_DIR = Path.home()
CURR_DIR = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve()) # Get current directory
RESULTS_DIR = f"{CURR_DIR}/examples/results/"
ISAACSIM_PYTHON = f'{HOME_DIR}/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh'
DEFAULT_POS = [[3.0, 3.0, 0.2],
               [7.5, 3.0, 0.2],
               [12.0,3.0, 0.2],
               [3.0, 7.5, 0.2],
               [7.5, 7.5, 0.2],
               [12.0,7.5, 0.2],
               [3.0, 12.0,0.2],
               [7.5, 12.0,0.2],
               [12.0,12.0,0.2]]


class Benchmark:
    def __init__(self, benchdict:dict={}) -> None:
        self.start_time =   datetime.now()
        self.y_arg =        Y_ARG
        self.scripts =      benchdict.get('scripts', ['examples/12_python_single_vehicle_gsl_benchmark.py'])
        self.envs =         benchdict.get('envs', [1])
        self.exp_start_id = benchdict.get('start_id', self.get_exp_start_id())
        self.pos_list =     benchdict.get('pos_list', DEFAULT_POS)
        self.pos_select =   benchdict.get('pos_select', range(len(DEFAULT_POS)))
        self.comment =      benchdict.get('comment', '')
        self.savetxt =      benchdict.get('save_txt', True)
        self.dry_run =      benchdict.get('dry_run', False)

        self.benchmark_list = self.get_benchmark_list()

    
    def get_exp_start_id(self) -> int:
        exp_dirs = [float(i) for i in os.listdir(RESULTS_DIR) if self.is_float(i)]
        exp_last = int(np.max(exp_dirs))
        return exp_last + 1


    def get_benchmark_list(self) -> list:
        benchmark_list = []
        exp_id = self.exp_start_id
        
        for _,script in enumerate(self.scripts):
            for _,env in enumerate(self.envs):
                for _,pos_idx in enumerate(self.pos_select):
                    benchmark_list.append([str(exp_id).zfill(3), script, env, self.pos_list[pos_idx]])
                    exp_id += 1

        return benchmark_list


    def save_benchmark_list(self) -> None:
        if self.dry_run:
            return
        
        if self.comment:
            filename = f'{datetime.now().strftime("%Y-%m-%d")}_{datetime.now().strftime("%H:%M:%S")}_{self.comment}'
        else:
            filename = f'{datetime.now().strftime("%Y-%m-%d")}_{datetime.now().strftime("%H:%M:%S")}'
        
        with open(f'{CURR_DIR}/{filename}.txt', 'w') as outfile:
            outfile.write('\n'.join(str(i) for i in self.benchmark_list))


    def execute(self, parameters) -> None:
        command = f'{ISAACSIM_PYTHON} {parameters[1]}'
        exp_id = str(parameters[0])
        env_id = str(parameters[2])
        pos =    parameters[3]

        print(f"RUN EXPERIMENT: {exp_id} --- ENV: {env_id} --- POSITION: {pos}")
        if not self.dry_run:
            os.system(f"{command} -- {exp_id} {env_id} '{pos}'")    


    def prompt(self) -> bool:
        if self.dry_run: print("DRY RUN!!!")
        if self.comment:
            print(f"Benchmarking: {self.comment}")
        print("The following experiments are in queue:")
        print(" exp_id                      script                         env_id   start_pos")
        print("--------------------------------------------------------------------------------")
        #     ['001', 'examples/12_python_single_vehicle_gsl_benchmark.py', 5, [7.5, 12.0,0.2]]
        print(*self.benchmark_list, sep = '\n')
        
        if self.y_arg == None:
            answer = input("Continue? y/n:")
        else:
            answer = self.y_arg
        
        if 'y' in answer or 'Y' in answer:
            return True
        else:
            return False


    def make_exp_dir(self, parameters) -> None:
        if self.dry_run:
            return
    
        exp_path = f"{CURR_DIR}/examples/results/{parameters[0]}/"
        if not os.path.exists(exp_path): # create experiment folder if it does not exist yet
            os.system(f"mkdir -p {exp_path}") 


    def run(self) -> None:
        if self.prompt():
            if self.savetxt: self.save_benchmark_list()

            for benchmark in self.benchmark_list:
                self.make_exp_dir(benchmark)
                self.execute(benchmark)
            print(f"Finished {len(self.benchmark_list)} experiments in {datetime.now() - self.start_time}")
        
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


    def gen_plots(self):
        # TODO: make function that automatically generates the requested plots
        pass


if __name__ == "__main__":
    bm = Benchmark(benchdict={
        "envs": [1,2,3,4,5,6],
        "save_txt": True,
        "comment": "Ecoli3D",
        "dry_run": False,
        })
    
    bm.run()
