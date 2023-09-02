"""
| File: gsl.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Definition of the GSL (Gas Source Localisation) class which is used as
| the base for all the GSL methods
"""
__all__ = ["GSL"]

import numpy as np

class GSL:
    """The base class for implementing a GSL method

    Attributes:

    """
    def __init__(self, gsl_type: str):
        """Initialize the Sensor class

        Args:
            gsl_type (str): A name that describes the type of GSL algorithm
        """

        # Set the GSL algorithm
        self._gsl_type = gsl_type


    def get_wp(self) -> np.ndarray:
        """
        Method that should be implemented by the class that inherits GSL method. 
        This is where the actual GSL algorithm is performed
        
        Args:
            none
        Returns:
            np.ndarray: A 3x3 numpy matrix with the next waypoint
        """
        pass


    def reset(self) -> None:
        """Method that gets called when the simulation and thus the controller is reset.
        
        Args:
            none
        Returns:
            np.ndarray: A 3x3 numpy matrix with the next waypoint
        """
        pass


    @property
    def gsl_type(self):
        """
        (str) A name that describes the type of GSL algorithm.
        """
        return self._gsl_type


