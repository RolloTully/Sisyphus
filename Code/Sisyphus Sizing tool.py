import numpy
from solarpy import irradiance_on_plane
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np

class Sun():
    def __init__(self, latitude, longditude, date = [2023,7,5]):
        self.Latitude = latitude
        self.Longditude = longditude
        self.Date = date
        self.h = 1 #Meters of altitude, assumes to be the cruise altitude
    def Panel_Irradience(self, Panel_Normal_Vector = np.array([0,0,-1])):
        '''Calcuated total solar intensity'''
        return np.array([irradiance_on_plane(Panel_Normal_Vector, self.h, datetime(self.Date[0], self.Date[1], self.Date[2], hour, minute), self.Latitude) for hour in range(0, 24) for minute in range(0, 60)])

class Battery(object):
    def __init__(self):
        '''
        The Battery is composed of reclaimed 21700s Li-ion cells
        each cell weights 68 grams and so the mass is stepped in sizes of 68 grams
        '''
        self.Cell_mass = 0.068
        self.Cell_Capacity_mah = 4900 #mah
        self.Cell_Nominal_voltage = 3.6 #volts
        self.Cell_Capacity = self.Cell_Nominal_voltage*(self.Cell_Capacity_mah/1000)*60*60
        self.Cell_Count = 1
        self.Mass = self.Cell_mass
    def Update(self, capacity):
        self.Cell_Count = np.ceil(capacity/self.Cell_Capacity)
        self.Energy_Capacity = self.Cell_Capacity*self.Cell_Count
        self.Mass = self.Cell_mass*self.Cell_Count


class Solar_Array(object):
    def __init__(self, cell_dimensions = np.array([0.125,0.125]), cell_mass = 0.007, cell_efficiency = 0.243, normal_vector = np.array([0,0,-1])):
        self.Latitude = 52.1307
        self.Longditude = 53.7837
        self.sun = Sun(self.Latitude, self.Longditude)
        self.Cell_Count = 0 #Number of cells in the array
        self.vnorm = normal_vector #Normal vector of the array
        self.Cell_Dimensions = cell_dimensions #The dimensions of each cell
        self.Cell_Mass = cell_mass #The Mass of each cell
        self.mass = 0 #Array Mass
        self.Cell_Efficiency = cell_efficiency #The Efficiency of each cell
        self.Cell_Area = np.prod(self.Cell_Dimensions) #The Area of the array
        '''Cell Performance characterisation, almost certain this is correct'''
        self.Cell_Total_Energy_Exposure = self.sun.Panel_Irradience()#An array with the surface solar irradiance every minute
        self.Cell_Gathered_Energy =  np.prod(self.Cell_Dimensions)*self.Cell_Total_Energy_Exposure*self.Cell_Efficiency #How much power the cell will collect over each minute of operation
        self.Cell_Daily_Production = np.trapz(self.Cell_Gathered_Energy*60)#This is how much energy the cell collects in a day under ideal conditions
        print("Per Cell Cell Daily Production", self.Cell_Daily_Production)
    def Update(self, capacity):
        self.Cell_Count = np.ceil(capacity/self.Cell_Daily_Production)
        self.Mass = self.Cell_Count*self.Cell_Mass
        self.Cell_Area = np.prod(self.Cell_Dimensions)*self.Cell_Count
        print("Capacity required: ",capacity," Cell gathered energy: ", self.Cell_Daily_Production," Cell count: ", self.Cell_Count, " Mass of Array: ",self.Mass, " Array Area: ",self.Cell_Area)

class Wing():
    def __init__(self, wing_density= 2.31, chord = 0.3, span = 6, efficiency = 0.7, lift_curve_slope = 2*np.pi, operating_cl = 0.2):
        self.Wing_Density = 2.32#kg/m^2
        self.Spar_density = 0.0838
        self.Aspect_Ratio = 40
        self.Span = 0.2
        self.Chord = self.Span/self.Aspect_Ratio
        self.Efficiency = 0.92 #This is assumed constant
        self.k = 1/(np.pi*self.Efficiency*self.Aspect_Ratio)
        self.Lift_Curve_Slope = 2*np.pi #Assumed Constant
        self.Operating_Cl = 1.0948 #Weirdly high but the maths dont lie, except when it does
        self.Cd0 = 0.01281
        self.Operating_LD_Ratio = 30
        self.Mass = 0
    def Power_Required(self, velocity, density, mass, empennage_area):
        '''Working'''
        self.Drag_coefficient = self.Cd0 + 0.074*((self.Chord*velocity*1.225)/1.48e-5)**-0.2 * (self.Operating_Cl**2)/(np.pi*self.Efficiency*self.Aspect_Ratio)

        self.Power = (self.Drag_coefficient/(self.Operating_Cl**(3/2)))*np.sqrt((2*(mass*9.81)**3)/(1.225*self.Wing_Area))

        return self.Power
    def Update(self, velocity, density, aircraft_mass, Required_Solar_Area):
        '''
        The Wing either needs to be big enought to fit enought solar panels, or be big enought to maintain flight whichever is bigger
        '''
        self.Lift_Limited_Area = (2*aircraft_mass*9.81)/(self.Operating_Cl*density*velocity**2)
        self.Solar_Limited_Area = Required_Solar_Area
        print("Solar Limited Area: ", self.Solar_Limited_Area, " Lift_Limited_Area: ",self.Lift_Limited_Area)
        if self.Solar_Limited_Area>self.Lift_Limited_Area:
            print("Solar Limited")
            self.Mass = self.Solar_Limited_Area*self.Wing_Density
            self.Wing_Area = self.Solar_Limited_Area
        else:
            print("Lift Limited")
            self.Mass = self.Lift_Limited_Area*self.Wing_Density
            self.Wing_Area = self.Lift_Limited_Area
        '''This is complicated as the chord and span cannot be continuily increased and must intead by incresed by whole cell widths'''
        self.Span = np.sqrt(self.Wing_Area*self.Aspect_Ratio)
        self.Mass = self.Mass + self.Span*self.Spar_density
        self.Chord = self.Span/self.Aspect_Ratio
        self.k = 1/(np.pi*self.Efficiency*self.Aspect_Ratio)
        print("Mainwing: Span: ",self.Span, "wing Chord: ",self.Span/self.Aspect_Ratio," Wing area: ", self.Wing_Area," Wing mass: ", self.Mass)

class Motor(object):
    def __init__(self):
        '''
        will operate best at high voltages
        should run
        '''
        self.Motor_Efficiency = 0.8 # This should be a function
        self.Mass = 0.1
    def Update(self, OPR):
        self.Mass = ((-2e-06)*((OPR/self.Motor_Efficiency)**2)+(0.0585*(OPR/self.Motor_Efficiency))+81.682)/1000 #This is an Empircical Relashonship
        print("Calcualted Motor Mass:",OPR, self.Mass)

class ESC():
    def __init__(self):
        '''
        Should be sized to be a S.F. of 1 or less
        this maximises Efficiency
        '''
        self.ESC_Efficiency = 0.97
        self.Mass = 0.200
    def Update(self, OPR):
        self.Mass = ((-2e-06)*((OPR/self.ESC_Efficiency)**2)+(0.0308*(OPR/self.ESC_Efficiency))+12.61)/1000 #This is an Empircical Relashonship
        print("Calculated ESC Mass: ",OPR,self.Mass)

class Cargo():
    def __init__(self, mass = 0.5):
        self.Mass = mass#kg of cargo

class Propulsion_System():
    def __init__(self):
        self.generic_motor = Motor()
        self.generic_esc = ESC()
        self.battery = Battery()
        self.solar_array = Solar_Array()
        self.Prop_efficiecy = 0.8 #Max efficiecy
        self.End_of_Night_Charge = 0.2 #At the end of the day(24hrs) we want to still have 20% of charge
        self.Mass = 0
    def Update(self,Power_Required, Velocity):
        self.Work_Rate = Power_Required/self.Prop_efficiecy
        self.Total_Work = self.Work_Rate*(60*60)*24 #The amount of energy expended everyday keeping the aircraft aloft
        self.Total_Work_Storage = self.Total_Work/(1-self.End_of_Night_Charge) #The amount we want to have stored to account fo bad weather
        print("Total work storage needed",self.Total_Work_Storage)
        self.battery.Update(self.Total_Work_Storage)
        self.generic_motor.Update(self.Work_Rate)
        self.generic_esc.Update(self.Work_Rate)
        self.solar_array.Update(self.Total_Work_Storage)
        self.Mass = self.generic_esc.Mass + self.generic_motor.Mass + self.battery.Mass + self.solar_array.Mass
        print(self.generic_esc.Mass, self.generic_motor.Mass, self.battery.Mass)

class Empennage(object):
    def __init__(self):
        self.Leaver_arm_density = 0.93#kg/m, 30 grams per meter
        self.Empennage_density = 2.31#kg/m^2
        self.Vertical_Tail_Volume_Coefficient = 0.02
        self.Horizontal_Tail_Volume_Coefficient = 0.5
        self.Mass = 0
        self.Leaver_Arm_length = 0
        self.Horizontal_Tail_Area = 0
        self.Vertical_Tail_Area = 0
    def Solve_emmenage(self, Wing_Area, Wing_Chord, Wing_Span):
        '''
        Think this is working
        Solves for the lever arm and tail area that minimises
        the mass of the tail arm and empennage
        V proud of this cos its a nice little derivative
        This will cause it to be very senstive to cg changes
        '''
        self.Leaver_Arm_length = np.sqrt((self.Empennage_density*Wing_Area*(self.Vertical_Tail_Volume_Coefficient*Wing_Span+self.Horizontal_Tail_Volume_Coefficient*Wing_Chord))/self.Leaver_arm_density)
        self.Horizontal_Tail_Area = (self.Horizontal_Tail_Volume_Coefficient*Wing_Chord*Wing_Area)/self.Leaver_Arm_length
        self.Vertical_Tail_Area = (self.Vertical_Tail_Volume_Coefficient*Wing_Span*Wing_Area)/self.Leaver_Arm_length
        self.Mass = self.Leaver_Arm_length*self.Leaver_arm_density+(self.Horizontal_Tail_Area+self.Vertical_Tail_Area)*self.Empennage_density

class Aircraft(object):
    def __init__(self):
        self.main_wing = Wing()
        self.battery = Battery()
        self.cargo = Cargo(1)
        self.empennage = Empennage()
        self.propulsion_system = Propulsion_System()
        self.Cruise_Speed = 8.6# needed to overcome likey weather in Llanbedr
    def Compute_Endurance(self):
        self.Enducrance  = self.battery.energy_capacity/self.Cruise_power()
    def Mass(self):
        return self.Wing.Mass+self.Batter.Mass+self.propulsion_system.mass
    def Size(self, Iterations):
        '''An inital estimate must be made this will be mega wrong but trust bro this gonna work'''
        self.old_mass = 0
        self.Mass = 6 #kg
        while (self.Mass-self.old_mass>0.00001):
            print("_________________________________________________________________Aircraft_________________________________________________________________")
            self.main_wing.Update(self.Cruise_Speed,1.225, self.Mass, self.propulsion_system.solar_array.Cell_Area)
            self.empennage.Solve_emmenage(self.main_wing.Wing_Area, self.main_wing.Chord, self.main_wing.Span)
            self.propulsion_system.Update(self.main_wing.Power_Required(self.Cruise_Speed, 1.225,0,self.Mass,self.empennage.Horizontal_Tail_Area+self.empennage.Vertical_Tail_Area),self.Cruise_Speed)
            self.old_mass = self.Mass
            self.Mass = self.main_wing.Mass + self.empennage.Mass + self.propulsion_system.Mass + self.cargo.Mass

            print("Aircraft:   Mass: ",self.Mass," Wing Mass: ", self.main_wing.Mass, " Empennage: ",  self.empennage.Mass," Propulsion system: ",  self.propulsion_system.Mass," Cargo: ", self.cargo.Mass)
            print("Battery Cell Count: ", self.propulsion_system.battery.Cell_Count, " Solar Cell Count: ",self.propulsion_system.solar_array.Cell_Count )
            print("Empennage: Lever Arm: ", self.empennage.Leaver_Arm_length, "Horizontal Tail Area: ", self.empennage.Horizontal_Tail_Area, "Vertical Tail Area: ", self.empennage.Vertical_Tail_Area, " Mass: ",self.empennage.Mass)
            print("Power Required",self.main_wing.Power_Required(self.Cruise_Speed, 1.225,0,self.Mass,self.empennage.Horizontal_Tail_Area+self.empennage.Vertical_Tail_Area))
            input(self.Mass)
        #self.State_of_Charge_Sim()
    '''
    def State_of_Charge_Sim(self):
        self.incident_solar = self.propulsion_system.solar_array.sun.Panel_Irradience()
        self.Solar_Power = self.incident_solar*self.propulsion_system.solar_array.Cell_Count*np.prod(self.propulsion_system.solar_array.Cell_Dimensions)*self.propulsion_system.solar_array.Cell_Efficiency
        self.Excess_power = self.Solar_Power - self.main_wing.Thrust_Required(self.Cruise_Speed, 1.225,0,self.Mass)
        self.Battery_Capacity = self.propulsion_system.battery.Cell_Count*self.propulsion_system.battery.Cell_Capacity
        print(np.min(np.cumsum(self.Excess_power)))
        #print(self.Battery_Capacity)
        plt.plot(self.Solar_Power)
        plt.plot(self.Excess_power)
        plt.plot(np.cumsum(self.Excess_power))
        #plt.plot((np.cumsum(self.Excess_power))+(self.Battery_Capacity))
        plt.show()
    '''
class Generic_Part():
    def __init__(self, mass):
        self.Mass = mass
        self.Position = position

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
        self.Sisyphus = Aircraft()
        self.mainloop()
    def mainloop(self):
        '''Runs the sizing for 10 iterations'''
        self.Sisyphus.Size(1)
        #self.Sisyphus.Solar_analysis()

if __name__ == "__main__":
    main()
