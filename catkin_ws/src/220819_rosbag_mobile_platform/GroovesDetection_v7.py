#!/usr/bin/env python3
""""" Grooves Detection Script"""
import numpy as np
import math
#import rospy
#import data_log

class GroovesDetection():

    def __init__(self):

        self.n_samples = 405
        self.rear_laser = self.n_samples * (0)
        self.front_laser =  self.n_samples * (0)
        self.degree_per_sample = math.radians(270)/810
        self.minimum_straight_distance = 0.75
        self.poly_order = 10
        self.threshold_flag = True
        self.dynamic_threshold_coeficient = 3
        self.fixed_threshold = 0.04
        self.stepsize_index = 6
        self.central_sample = (self.n_samples - 1)/2
        self.rear_dis = []
        self.front_dis = []
        self.rear_fitted_curve_index = []
        self.rear_fitted_curve = []
        self.front_fitted_curve_index = []
        self.front_fitted_curve = []
        self.data_x = []
        self.fitted_curve = []
        self.dis = []
        self.grooves = None
        self.data_x = None
        self.rear_grooves = None
        self.front_grooves = None

    def _detect_grooves(self, data):
        
        if data:

            readings_array = np.array([range(len(data)), data])
            valid_readings_filter = readings_array[1]*np.cos(self.degree_per_sample *(readings_array[0]- self.n_samples/2)) > self.minimum_straight_distance
            valid_readings = readings_array[:,valid_readings_filter]
                
            self.data_x = valid_readings[0]
            data_y = valid_readings[1]
            data_length = len(self.data_x)
            
            if data_length > self.poly_order:

                poli_coefficients, residual, _, _, _  = np.polyfit(self.data_x, data_y, self.poly_order,full=True)               
                fitted_function = np.poly1d(poli_coefficients)

                self.fitted_curve = fitted_function(self.data_x)

                deviation = np.sqrt(residual/data_length)
                if self.threshold_flag:

                    threshold = self.fixed_threshold

                else:

                    threshold = self.dynamic_threshold_coeficient * deviation

                deviation_dis_filter = data_y - self.fitted_curve >= threshold
                possible_grooves = valid_readings[ : , deviation_dis_filter]
                
                group_grooves = np.split(possible_grooves[0], np.where(np.diff(possible_grooves[0]) > self.stepsize_index)[0]+1)
                index_list = []
                number_possibles_groups = len(group_grooves)
                if number_possibles_groups == 2 or number_possibles_groups == 1:

                    try:

                        index = group_grooves[0][-1] +1
                        if index > 404:

                            index = 404
                        index_list.append(index)

                        if number_possibles_groups == 2:
                            index = group_grooves[1][0]  -1
                            if index < 0:

                                index = 0
                            index_list.append(index)
       
                        grooves = readings_array[:, np.array(index_list,dtype=int)]
                        self.dis = grooves[1]*np.sin(self.degree_per_sample *(-grooves[0] + self.n_samples/2))

                        return grooves
                         
                    except:

                        print("Error")

                else:

                    print(str(number_possibles_groups) + " was detected in a sensor !!!!!!")

        self.dis = np.array([])
        self.data_x = np.array([])
        self.fitted_curve = np.array([])
        return np.array([])


    def update_sensor_data(self, rear_laser, front_laser):

        self.rear_laser = rear_laser
        self.front_laser = front_laser

        self.rear_grooves = self._detect_grooves(self.rear_laser)
        self.rear_dis = self.dis
        self.rear_fitted_curve_index = self.data_x
        self.rear_fitted_curve = self.fitted_curve

        self.front_grooves = self._detect_grooves(self.front_laser)
        self.front_dis = self.dis
        self.front_fitted_curve_index = self.data_x
        self.front_fitted_curve = self.fitted_curve

        return self.rear_dis, self.front_dis


