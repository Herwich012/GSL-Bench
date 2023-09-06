"""
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
"""
import carb
import numpy as np

class StopCondition():
    def __init__(self, 
                 time:float = None,
                 source_pos:np.ndarray = None, 
                 distance2src:float = None) -> None:
        self.stop_time = time # [s] max runtime
        self.src_pos = source_pos 
        self.stop_distance2src = distance2src # [m] max distance to the source to invoke the stop condition
        self.type = None


    def stop_time_cond(self, time) -> bool:
        stop = False
        
        if time > self.stop_time:
            stop = True
            carb.log_warn(f"Exceeded stop time ({round(self.stop_time,2)}s), stopping simulation...")

        return stop


    def stop_distance2src_cond(self, time, pos,) -> bool:
        stop = False
        # if the distance between robot and source is smaller than the stop distance
        # and the time is more than 5 sec, because of a reset bug
        if time > 5.0 and np.linalg.norm((pos - self.src_pos)) < self.stop_distance2src:
            stop = True
            carb.log_warn(f"Within stop distance ({round(self.stop_distance2src,2)}m), stopping simulation...")

        return stop


    def stop_gas_concentration_cond(self) -> bool:
        stop = False
        # TODO implement gas concentration stop condition
        return stop


    def get(self, time_current=None, pos_current=None) -> bool:
        if self.stop_time is not None and self.stop_time_cond(time_current):
            self.type = "time"
            return True
        
        if self.stop_distance2src is not None and self.stop_distance2src_cond(time_current,pos_current):
            self.type = "dist2src"
            return True 

        return False
