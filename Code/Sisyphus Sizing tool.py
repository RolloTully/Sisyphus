import numpy
from solarpy import irradiance_on_plane
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
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
    def __init__(self,latitude, longditude, date = [2023,7,5]):
        self.Latitude = latitude
        self.Longditude = longditude
        self.Date = date
        self.h = 100 #Meters of altitude, assumes to be the cruise altitude
    def Total_Incident_Energy(self,Panel_Normal_Vector = np.arrya([0,0,-1])):
        return np.sum(self.Panel_Irradience(Panel_Normal_Vector))

    def Panel_Irradience(self):
        return np.array([irradiance_on_plane(Panel_Normal_Vector, self.h, datetime(date[0], date[1], date[2], hour, minute), self.Latitude) for minute in range(0, 60) for hour in range(0, 24)])



class Battery(object):
    def __init__(self, Battery_Energy_Density, Battery_Mass):
        self.energy_density = Battery_Energy_Density #Mj/kg
        self.mass = Battery_Mass
        self.energy_capacity = self.energy_density*self.mass
    def update(self):
        self.mass = self.energy_capacity/self.energy_density

    def Set_capacity(self, cap):
        self.energy_capacity = cap
        self.update()

class Solar_Cell(object):
    def __init__(self, Cell_Dimensions, Cell_mass, Cell_Efficiency, Normal_Vector = np.array([0,0,-1])):
        self.sun = Sun(self.Latitude, self.Longditude, )#Yes this is horribly memory inefficent, do i care, Yes. do I care to fix it No
        self.vnorm = Normal_Vector
        self.Dimensions = Cell_Dimensions
        self.mass = Cell_mass
        self.Cell_Efficiency = Cell_Efficiency


class Power_Model():
    def __init__(self):
        pass

class Wing():
    def __init__(self):
        self.Wing_Density = 2.31#kg/m^2
        self.Chord = 0.3
        self.Span = 6
        self.Efficiency = 0.7 #This is assumed constant
        self.Lift_Curve_Slope = 2*np.pi #Assumed Constant
    def Lift_Required(self):
    def Drag_induced(self):


class Motor(object):
    def __init__(self):
        '''
        will operate best at high voltages
        should run
        '''
        self.Motor_Efficiency = 0.8 # This should be a function
        self.mass = 0.1
    def Update_Mass(self, OPR):
        self.mass = (-2e-05)*(OPR**2)+(0.0585*OPR)+81.682 #This is an Empircical Relashonship
    def Power_Required(self):



class ESC():
    def __init__(self):
        '''
        Should be sized to be a S.F. of 1 or less
        this maximises Efficiency
        '''
        self.ESC_Efficiency = 0.97
        self.mass = 0.200
    def Update_Mass(self, OPR):
        self.mass = (-2e-06)*(OPR**2)+(0.0308*OPR)+12.61 #This is an Empircical Relashonship
class Cargo():
    def __init__(self):
        self.Mass = 0.5#kg of cargo
class Propulsion_System():
    def __init__(self):
        self.generic_motor = Motor()
        self.generic_esc = ESC()
        self.Prop_efficiecy = 0.7 #Max efficiecy

class Aircraft(object):
    def __init__(self):
        self.main_wing = Wing()
        self.battery = Battery()
        self.propulsion_system = Propulsion_System()
        self.Solar_Array = [for _ in range(0,)]
        self.Cruise_Speed = 15# needed to overcome likey weather in Llanbedr
    def Cruise_power(self):
        self.Drag = (1/2)1.225*
        return
    def Compute_Endurance(self):
        self.Enducrance  = self.battery.energy_capacity/self.Cruise_power()

class Sensitivity_Analysis():
    '''
    Each Parameteer is changed by a tiny amount and the effect is observed
    if the effect is benificial then it is kept and the first step is repeated until convergence

    in this case we aim to increase Loiter time
    '''

    def __init__(self):


class main():
    def __init__(self):
        '''Location of RAF Llanbedr'''
        self.Latitude = 52.1307
        self.Longditude = 3.7837

        '''Defines object instances'''

        self.solar_cell  = Solar_Cell(np.array([125,125]),0.007,0.3)#Defines a C60 Monocrysataline Solar Cell

        self.Sisyphus = Aircraft()
        self.mainloop()
    def mainloop(self):
        self.irradiance_array  = self.sun.Compute_total_irradience()*60*60
        print(self.irradiance_array)
        self.cumulative_irradiace = np.cumsum(self.irradiance_array)
        self.total_absorbidance_array = self.cumulative_irradiace*0.23*0.585
        self.total_power_excess = self.total_absorbidance_array-70*60*60
        print(np.sum(self.total_power_excess))
        plt.plot(self.cumulative_irradiace)
        plt.plot(self.irradiance_array)
        plt.plot(self.total_absorbidance_array)
        plt.plot(self.total_power_excess)
        plt.show()



if __name__ == "__main__":
    main()
