#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 19:48:46 2022

@author: gpdas
"""
import math
import matplotlib.pyplot as pyplt

class SecondOrderSystem(object): 
    """definition of a second order system - cascade of two first order systems (low pass filters)
    H[z] = a / (1 – b z^-1) * c / (1 – d z^-1)= a0c0 / (1 – (b+d) z^-1  + (b*d) z^-2  )
    """
    def __init__(self, d1, d2):
        """initialise the system
        
        Args:
            d1 -- decay_1 sample number (filter_1 output will decay/rise exactly by 63.2 % at this sample)
            d2 -- decay_2 sample number (filter_2 output will decay/rise exactly by 63.2 % at this sample)
        """
        if d1 > 0:
            e1 = -1.0 / d1
            decay_1 = math.exp(e1)
        else: 
            decay_1 = 0
        
        if d2 > 0:
            e2 = -1.0 / d2
            decay_2 = math.exp(e2)
        else: 
            decay_2 = 0
        
        a = 1.0 - decay_1    # b = decay_1
        c = 1.0 - decay_2    # d = decay_2
        self.ac = a * c
        self.b_plus_d = decay_1 + decay_2
        self.bd = decay_1 * decay_2
        self.prev_prev_output_value = 0
        self.prev_output_value = 0

    def __call__(self, input_value): 
        output_value = self.ac * input_value + \
                       self.b_plus_d * self.prev_output_value -\
                       self.bd * self.prev_prev_output_value
        self.prev_prev_output_value = self.prev_output_value
        self.prev_output_value = output_value
        return output_value
    
    def reset(self):
        self.prev_prev_output_value = 0
        self.prev_output_value = 0

class PIDControlBlock(object):
    """PID Controller
    """
    def __init__(self, k_p, k_i, k_d, d_t=1.0):
        """Initialise PID
        
        Args:
            k_p -- proportional gain
            k_i -- integral gain
            k_d -- derivative gain
            d_t -- sampling time
        """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.d_t = d_t
        self.prev_err = 0
        self.prev_prev_err = 0
        self.prev_prev_prev_err = 0
        self.sum_err = 0
 
    def __call__(self, err): 
        p_out = self.k_p * err
        self.sum_err +=  err
        i_out = self.k_i * self.d_t * self.sum_err
        d_out = (self.k_d / self.d_t) * (err - self.prev_err)
        output = p_out + \
                 i_out + \
                 d_out
        self.prev_err = err
        return output

class ClosedLoopSystem(object):
    def __init__(self, controller, plant_process):
        self.plant_process = plant_process
        self.controller = controller
        self.plant_output_feedback = 0
 
    def __call__(self, ref_input):
        err = ref_input - self.plant_output_feedback
        controller_output = self.controller(err)
        plant_output = self.plant_process(controller_output)
        self.plant_output_feedback = plant_output
        return plant_output

if __name__ == "__main__":
    ref_input = 100
    n_samples = 1000
    
    plant_process = SecondOrderSystem(250, 100)
    system_output = []
    for i in range(n_samples):
        system_output.append(plant_process(ref_input))
    
    plant_process.reset()
    pid_controller = PIDControlBlock(5, 0.0143, 356.25)
    controlled_process = ClosedLoopSystem(pid_controller, plant_process)
     
    controlled_system_output = []
    for i in range(n_samples):
        controlled_system_output.append(controlled_process(ref_input))
    
    fig1 = pyplt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot([ref_input for i in range(n_samples)])
    p11 = ax1.plot()
    p12 = ax1.plot(system_output)
    ax1.set_title("uncontrolled_process")
    fig1.show()
    
    fig2 = pyplt.figure()
    ax2 = fig2.add_subplot(111)
    p21 = ax2.plot([ref_input for i in range(n_samples)])
    p22 = ax2.plot(controlled_system_output)
    ax2.set_title("controlled_process_1")
    fig2.show()

    plant_process.reset()
    pid_controller = PIDControlBlock(5, 0.0343, 356.25)
    controlled_process = ClosedLoopSystem(pid_controller, plant_process)
     
    controlled_system_output = []
    for i in range(n_samples):
        controlled_system_output.append(controlled_process(ref_input))

    fig3 = pyplt.figure()
    ax3 = fig3.add_subplot(111)
    p31 = ax3.plot([ref_input for i in range(n_samples)])
    p32 = ax3.plot(controlled_system_output)
    ax3.set_title("controlled_process_2")
    fig3.show()