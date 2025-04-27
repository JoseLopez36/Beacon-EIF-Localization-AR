#!/usr/bin/env python

class EIFDroneModel:
    """ 
    Modelado de estado:
    g : Función no lineal de transición de estado
    h : Función no lineal de medición
    Para la linealización de las funciones para su aproximación a forma gaussiana mediante aproximación de Taylor - orden 1:
    G : Jacobiano de g 
    H : Jacobiano de h
    """  
    def __init__(self):
     
        pass

    
    def g_function(self, u , mu):
        pass

    def h_function(self, mu_pred):
        pass

    def G_jacobian(self, g):

        pass

    def H_jacobian(self, h):
        pass
        




