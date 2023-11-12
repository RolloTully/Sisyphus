import numpy
from solarpy import
class Standard_Atmosphere():
    def __init__(self):
        self.g = 9.8065
        self.R = 287.05287
        self.Gamma = 1.4
        self.Lapse_0 = -0.0065
        self.Lapse_11 = 0
        self.Lapse_20 = 0.001
        self.Temp_0 = 288.15
        self.Temp_11 = 216.65
        self.Temp_20 = 216.65
        self.Pres_0 = 101325
        self.Pres_11 = 22632.559
        self.Rho = 1.225
    def get_AtmosProperties(self, h):
        '''Computes Static pressure, Temperature, density, and the local speed of sound'''
        if 0.0 <= h <= 11e3:
            self.Local_Temperature = self.T0+self.L0*(h-0)
        elif 11e3 < h <= 20e3:
            self.Local_Temperature = self.T11+self.L11*(h-11)
        elif 20e3 < h <= 32e3:
            self.Local_Temperature = self.T20+self.L20*(h-20)
        elif 32e3< h:
            print("Height out of range")
        if 0.0 <= h <= 11e3:
            self.Local_Pressure = self.P0*(1+(self.L0/self.T0)*h)**(-self.g/(self.R*self.L0))
        elif 11e3 < h <= 20e3:
            self.Local_Pressure = self.P11*e**((-self.g/(self.R*self.T11))*(h-11e3))
        elif 20e3 < h <= 32e3:
            assert "Height out of range"
            pass
        elif 32e3 < h:
            print("Height out of range")
        self.Local_Desnity = self.Local_Pressure/(self.R*self.Local_Temperature)
        self.a = (self.gamma*self.R*self.Local_Temperature)**0.5
        return self.Local_Temperature, self.Local_Pressure, self.Local_Desnity, self.a

class Sun():
    def __init__(self, lat, long):
        self.Latitude = 52.1307
        self.Longditude = 3.7837
    def Compute
class Battery(object):
    def __init__(self):
        self.energy_density = 0.72e6 #Mj/kg
        self.energy_capacity = 0
        self.mass = 0
    def update(self):
        self.mass = self.energy_capacity/self.energy_density

    def Set_capacity(self, cap):
        self.energy_capacity = cap
        self.update()

class Solar_Cell(object):
    def __init__(self):
        self.vnorm = np.array([0,0,-1])
        self.Dimensions =
        self.mass =


class Power_Model():
    def __init__(self):

class Wing():
    def __init__(self):
        self.Chord =
        self.Span =
        self.Efficiency =


class main():
    def __init__(self):
        self.Latitude = 52.1307
        self.Longditude = 3.7837
        self.sun = Sun(self.Latitude, self.Longditude)
class Aircraft(object):
    def __init__(self):
        self.main_wing = Wing()
        self.battery = Battery()
