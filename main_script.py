import pygame
import os
import matplotlib.pyplot as plt
import numpy as np
import math
from tkinter import Tk, Text, TOP, BOTH, X, N, LEFT, RIGHT
from tkinter.ttk import Frame, Label, Entry, Button
  
pygame.init()
clock = pygame.time.Clock()
# size of the display window 
X_size = 1200
Y_size = 650
scrn = pygame.display.set_mode((X_size,Y_size))

class SimpleDialog(Frame):
    '''SimpleDialog is defined to ask user to input initial position of robot through a dialogue box'''
    def __init__(self):
        super().__init__()
        self.output1 = ""
        self.output2 = ""
        self.output3 = ""
        self.initUI()

    def initUI(self):
        self.master.title("Initial position")
        self.pack(fill=BOTH, expand=True)

        frame1 = Frame(self)
        frame1.pack(fill=X)

        lbl1 = Label(frame1, text="x0", width=6)
        lbl1.pack(side=LEFT, padx=5, pady=10)

        self.entry1 = Entry(frame1, textvariable=self.output1)
        self.entry1.pack(fill=X, padx=5, expand=True)

        frame2 = Frame(self)
        frame2.pack(fill=X)

        lbl2 = Label(frame2, text="y0", width=6)
        lbl2.pack(side=LEFT, padx=5, pady=10)

        self.entry2 = Entry(frame2, textvariable =self.output2)
        self.entry2.pack(fill=X, padx=5, expand=True)

        frame3 = Frame(self)
        frame3.pack(fill=X)

        lbl3 = Label(frame3, text="theta0", width=6)
        lbl3.pack(side=LEFT, padx=5, pady=10)

        self.entry3 = Entry(frame3, textvariable = self.output3)
        self.entry3.pack(fill=X, padx=5, expand=True)

        frame4 = Frame(self)
        frame4.pack(fill=X)

        # Command tells the form what to do when the button is clicked
        btn = Button(frame4, text="Submit", command=self.onSubmit)
        btn.pack(padx=5, pady=10)

    def onSubmit(self):

        self.output1 = self.entry1.get()
        self.output2 = self.entry2.get()
        self.output3 = self.entry3.get()
        self.quit()

''' to_pygame function coverts the real world coordinate to the coordinate system of pygame'''
def to_pygame(coords, height, width):
    """Convert coordinates into pygame coordinates (center of the display is the origin)."""
    return (coords[0]+(width/2), (height/2) - coords[1]) 

def F(x,y,theta):
    # function F() converts q(x,y,theta) into a,alpha 
    ''' theta_d represents angle of tangent to circle at any point (x,y)
        r represnts the radius of the circle
        a represents arch length from origin to (x,y) along the circle 
        alpha represents orientation error and value lies between (-pi, pi]'''

    if (x == 0 and y == 0):
        theta_d = 0     # at origin
    elif(x == 0 and y != 0):
        theta_d = 180  
    else:
        theta_d = 2*math.degrees(math.atan(y/x))
        if (theta_d == -180):theta_d = 180
        
    if(y == 0): 
        r = 0
    else:
        r = (x**2+y**2)/(2*y)

    a = r*math.radians(theta_d) 
    if(y == 0): a = 0
    
    e = theta - theta_d
    # limiting alpha within (-pi,pi]
    if (e > -180 and e <= 180):
        alpha = e
        n = 0
    else:
        temp = abs(e) - 180
        rang = temp // 360
        n = np.sign(e)*(rang+1)
        alpha = e - 2*180*n
        if (alpha == -180): alpha = 180
    return a,alpha,theta_d,n

def control_inputs(q,a,alpha,theta_d,n,k,gamma):
    '''function control_inputs calculates the required linear and angular velocities of the robot
        Inputs:
        q = current state of the robot represenred by (x,y,theta)
        a = arc length
        alpha = orientation error
        theta_d = tangent angle
        n = integer value to keep alpha within (-pi,pi]
        k, gamma = control variables'''

    if(q[0] == 0):
        # obtaining control inputs at discountinous surface D (Y-axis)
        v = -gamma*(-math.cos(math.radians(q[2]))+(math.pi/2)*math.sin(math.radians(q[2])))*(math.pi/2)*abs(q[1])
        w1 = -gamma*math.pi*math.cos(math.radians(q[2]))*(-math.cos(math.radians(q[2]))+(math.pi/2)*math.sin(math.radians(q[2])))*np.sign(q[1])
        w2 = -k*(math.radians(q[2])-math.pi-2*math.pi*n)
        omega = w1 + w2
    else:
        # obtaining control inputs at continuous surface
        beta = q[1]/q[0]
        b11 = math.cos(math.radians(q[2]))*(math.radians(theta_d)/beta -1)
        b12 = math.sin(math.radians(q[2]))*(((math.radians(theta_d)/2)*(1-(1/beta**2)))+1/beta)
        b1 = b11 + b12

        b21 = math.cos(math.radians(q[2]))*((2*beta)/((1+beta**2)*q[0]))
        b22 = math.sin(math.radians(q[2]))*(-2/((1+beta**2)*q[0]))
        b2 = b21 + b22

        v = -gamma*b1*a
        omega = -b2*v - k*math.radians(alpha)
    return v,omega

def write_info(x,y,theta):
    '''function to display the information about the current state of the robot'''
    font = pygame.font.SysFont('TimesNewRoman.ttf',25)
    txt = f"x = {round(x,2)}, y = {round(y,2)}, theta = {round(theta,2)}°"
    text = font.render(txt,True,(0,0,0),(255,255,255))
    textRect = text.get_rect()
    textRect.center = (1040,30)
    scrn.blit(text,textRect)

'''main program begins'''
pygame.init()
# initial position of the robot 
root = Tk()
root.geometry("250x170+300+300")
prompt = SimpleDialog()
root.mainloop()
user_input = (prompt.output1, prompt.output2, prompt.output3)
try:
    root.destroy()
except:
    pass
x0 = float(user_input[0])
y0 = float(user_input[1])
theta0 = float(user_input[2])

# assigned values of the control variables
gamma = 1.35
k = 4.60

q = [x0, y0, theta0]
print (q)
dt = 0.03
x_list = []
y_list = []
theta_list = []
#v_list = []
#w_list = []
time_list = []
trail_set = []
q_dot = [0, 0, 0]

# loading the background and the robot image for pygame
file_dir = os.path.dirname(os.path.abspath(__file__))
background_file = os.path.join(file_dir, "white.jpg")
robot_file = os.path.join(file_dir, "wheeled_robot.png")
background = pygame.image.load(background_file)
#scrn.blit(background,(0,0))
robot = pygame.image.load(robot_file)
robot0 = robot

t = 1
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    x_list.append(q[0])
    y_list.append(q[1])
    theta_list.append((q[2]))
    time_list.append(dt*t)

    # displaying the white background and the transformed robot along with information on its state
    scrn.blit(background,(0,0))
    pygame.draw.line(background,(0,0,0),(X_size/2,0),(X_size/2,Y_size))
    pygame.draw.line(background,(0,0,0),(0,Y_size/2),(X_size,Y_size/2))
    rect0 = robot0.get_rect(center = (q[0],q[1]))
    robot1= pygame.transform.rotate(robot0, q[2]-90)
    scrn.blit(robot1,(to_pygame((rect0[0],rect0[1]+44),Y_size,X_size)))
    write_info(q[0],q[1],q[2])

    # creating path of the robot for visualization
    for i in range(0,len(trail_set)-1):
        pygame.draw.line(background,(255,0,0),(trail_set[i][0],trail_set[i][1]),(trail_set[i+1][0],trail_set[i+1][1]))
    if trail_set.__sizeof__()>300:
        trail_set.pop(0)
    trail_set.append(to_pygame((q[0],q[1]),Y_size,X_size))

    a,alpha,theta_d,n = F(q[0],q[1],q[2]) # function call to find a and alpha
    v,omega = control_inputs(q,a,alpha,theta_d,n,k,gamma) # function call to obtain control inputs
    #v_list.append(v)
    #w_list.append(omega)

    # updating the values of state variables using the calculated linear and angular velocities
    q_dot = [v*math.cos(math.radians(q[2])),v*math.sin(math.radians(q[2])),omega]
    q[0]=q[0]+q_dot[0]*dt
    q[1]=q[1]+q_dot[1]*dt
    q[2]=math.degrees(math.radians(q[2])+q_dot[2]*dt)
    t = t + 1

    pygame.display.update()
    clock.tick(10)
pygame.quit()


# plotting the graph of state variables with respect to the time
plt.plot(time_list,x_list, color = 'g', label = 'x')
plt.plot(time_list,y_list, color = 'r', label = 'y')
plt.plot(time_list,theta_list, color = 'b', label = 'theta')
#plt.plot(x_list, y_list, color = 'b')
#plt.plot(time_list,v_list, color = 'b', label = 'v')
#plt.plot(time_list,w_list, color = 'r', label = 'w')
plt.xlabel("Time")
plt.ylabel("x, y, theta(degrees)")
plt.legend()
plt.grid()
plt.show()
