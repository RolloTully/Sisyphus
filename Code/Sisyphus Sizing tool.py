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
    def __init__(self):
        '''
        The Battery is composed of reclaimed 21700s Li-ion cells
        each cell weights 68 grams and so the mass is stepped in sizes of 68 grams
        '''
        self.Cell_mass = 0.068
        self.Cell_Capacity_mah = 4900 #mah
        self.Cell_Nominal_voltage = 3.6 #volts
        self.Cell_Capacity = self.Cell_Nominal_voltage*(self.Cell_Capacity_mah/1000)
        self.Cell_Count = 1
        self.Mass = 0
        self.update(self.Cell_Capacity_mah)
    def Update(self, capacity):
        self.Cell_Count = np.ceil(capacity/self.Cell_Capacity)
        self.Energy_Capacity = self.Cell_Capacity*self.Cell_Count
        self.Mass = self.Cell_mass*self.Cell_Count

class Solar_Array(object):
    def __init__(self, Cell_Dimensions, Cell_mass, Cell_Efficiency, Normal_Vector = np.array([0,0,-1])):
        self.sun = Sun(self.Latitude, self.Longditude, )#Yes this is horribly memory inefficent, do i care, Yes. do I care to fix it No
        self.vnorm = Normal_Vector
        self.Dimensions = Cell_Dimensions
        self.mass = Cell_mass
        self.Cell_Efficiency = Cell_Efficiency
    def Update(self):


class Wing():
    def __init__(self, wing_density= 2.31, chord = 0.3, span = 6, efficiency = 0.7, lift_curve_slope, operating_cl):
        self.Wing_Density = 2.31#kg/m^2
        self.Aspect_Ratio = 25
        self.Span = 6
        self.Chord = self.Span/self.Aspect_Ratio
        self.Efficiency = 0.7 #This is assumed constant
        self.Lift_Curve_Slope = 2*np.pi #Assumed Constant
        self.Operating_Cl = 0.98 #Weirdly high but the maths dont lie, except when it does
        self.Operating_LD_Ratio = 50
        self.Mass = 0
    def Drag_induced(self):
        pass
    def Thrust_Required(self,velocity, desnity):
        return (1/2)*desnity*self.Wing_Area*(self.Operating_Cl/self.Operating_LD_Ratio)*velocity**2
    def Update(self, velocity, density, aircraft_mass, Required_Solar_Area):
        '''
        The Wing either needs to be big enought to fit enought solar panels, or be big enought to maintain flight whichever is bigger
        '''
        self.Lift_Limited_Area = (2*aircraft_mass)/(Operating_Cl*density*velocity**2)
        self.Solar_Limited_Area = Required_Solar_Area
        if self.Solar_Limited_Area>self.Lift_Limited_Area:
            self.Mass = self.Solar_Limited_Area*self.Wing_Density
            self.Wing_Area = self.Solar_Limited_Area
        else:
            self.Mass = self.Lift_Limited_Area*self.Wing_Density
            self.Wing_Area = self.Lift_Limited_Area
        self.Span = np.sqrt(self.Wing_Area*self.Aspect_Ratio)


class Motor(object):
    def __init__(self):
        '''
        will operate best at high voltages
        should run
        '''
        self.Motor_Efficiency = 0.8 # This should be a function
        self.Mass = 0.1
    def Update(self, OPR):
        self.Mass = (-2e-05)*((OPR/self.Motor_Efficiency)**2)+(0.0585*(OPR/self.Motor_Efficiency))+81.682 #This is an Empircical Relashonship

class ESC():
    def __init__(self):
        '''
        Should be sized to be a S.F. of 1 or less
        this maximises Efficiency
        '''
        self.ESC_Efficiency = 0.97
        self.Mass = 0.200
    def Update_Mass(self, OPR):
        self.Mass = (-2e-06)*((OPR/self.ESC_Efficiency)**2)+(0.0308*(OPR/self.ESC_Efficiency))+12.61 #This is an Empircical Relashonship

class Cargo():
    def __init__(self, mass = 0.5):
        self.Mass = mass#kg of cargo

class Propulsion_System():
    def __init__(self):
        self.generic_motor = Motor()
        self.generic_esc = ESC()
        self.battery = Battery()
        self.solar_array =
        self.Prop_efficiecy = 0.7 #Max efficiecy
        self.Mass = 0
    def Update(self,Thrust_Required, Velocity):
        self.Work_Rate = Thrust_Required*Velocity
        self.Total_Work = self.Work_Rate*(24*60*60) #The amount of energy expended everyday keeping the aircraft aloft
        self.Total_Work_Storage = self.Total_Work/0.8 #The amount we want to have stored to account fo bad weather
        self.battery.Update(self.Total_Work_Storage)
        self.generic_motor.Update(self.Work_Rate/0.7)
        self.generic_esc.Update(self.Work_Rate/0.7)
        self.Mass = self.generic_esc.Mass + self.generic_motor.Mass + self.battery.Mass +

class Empennage(object):
    def __init__(self):
        self.Leaver_arm_density = 0.03#kg/m, 30 grams per meter
        self.Empennage_density = 2.31#kg/m^2
        self.Vertical_Tail_Volume_Coefficient = 0.02
        self.Horizontal_Tail_Volume_Coefficient = 0.5

    def Solve_emmenage(self, Wing_Area, Wing_Chord, Wing_Span):
        '''
        Solves for the lever arm and tail area that minimises
        the mass of the tail arm and empennage
        V proud of this cos its a nice little derivative
        This will cause it to be very senstive to cg changes
        '''
        self.Leaver_Arm_length = np.sqrt((self.Empennage_density*Wing_Area*(self.Vertical_Tail_Volume_Coefficient*Wing_Span+self.Horizontal_Tail_Volume_Coefficient*Wing_Chord))/self.Leaver_arm_density)
        self.Horizontal_Tail_Area = (self.Vertical_Tail_Volume_Coefficient*Wing_Chord*Wing_Area)/self.Leaver_Arm_length
        self.Vertical_Tail_Area = (self.Vertical_Tail_Volume_Coefficient*Wing_Span*Wing_Area)/self.Leaver_Arm_length
        self.Mass = self.Leaver_Arm_length*self.Leaver_arm_density+(self.Horizontal_Tail_Area+self.Vertical_Tail_Area)*self.Empennage_density

class Aircraft(object):
    def __init__(self):
        self.main_wing = Wing()
        self.battery = Battery()
        self.cargo = Cargo(0.5)
        self.propulsion_system = Propulsion_System()
        self.empennage = Empennage()
        self.Solar_Array = [for _ in range(0,)]
        self.Cruise_Speed = 15# needed to overcome likey weather in Llanbedr
    def Compute_Endurance(self):
        self.Enducrance  = self.battery.energy_capacity/self.Cruise_power()
    def Mass(self):
        return self.Wing.Mass+self.Batter.Mass+self.propulsion_system.mass
    def Size(self, Iterations):
        '''An inital estimate must be made this will be mega wrong but trust bro this gonna work'''
        self.Mass = 1 #kg
        self.main_wing.Update(self.Cruise_Speed, self.Mass, 0.125**2)
        self.empennage.Solve_emmenage(self.main_wing.Wing_Area, self.main_wing.Chord, self.main_wing.Span)
        self.propulsion_system.Update(self.main_wing.Thrust_Required(self.Cruise_Speed, 1.225),self.Cruise_Speed)
        self.Mass = self.main_wing.Mass + self.empennage.mass + self.propulsion_system.mass + self.cargo.mass


class Sensitivity_Analysis():
    '''
    Each Parameteer is changed by a tiny amount and the effect is observed
    if the effect is benificial then it is kept and the first step is repeated until convergence

    in this case we aim to increase Loiter time
    '''

    def __init__(self):
        pass


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
        '''Runs the sizing for 10 iterations'''
        self.Sisyphus.Size(10)


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
