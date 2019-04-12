import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

global pi
global femur
global tibia
global coxa
global n_steps
pi = np.pi

# %%%%%%%%%%%%%%%
# %  1 -- 0 %  %
# %  |    | %  %
# %  2 -- 3 %  %
# %%%%%%%%%%%%%%%
# # y    % %
# % |    % %
# % o--x % %

def OscillatorFunc():

    # Initilization
    w12 = 0.5
    w14 = 0.5
    w23 = 0.5
    w43 = 0.5
    w =  np.array([[0.0, w12, 0.0, w14], # row is one vector
        [w12, 0.0, w23, 0.0],
        [0.0 , w23, 0.0, w43],
        [w14 , 0.0, w43, 0.0]])

    w_f = np.array([10.0,10.0,10.0,10.0])
    phi = np.array([pi ,0.0 , pi, 0.0]) # phase difference

    wp13 = -pi/2
    wp14 = -pi/2
    wphi23 = -pi/2
    wphi43 = -pi/2
    wphi = np.array([[0.0, wp13, 0.0, wp14],
            [wp13, 0.0, wphi23, 0.0],
            [0.0, wphi23, 0.0, wphi43],
            [wp14, 0.0, wphi43, 0.0]])


    dot_phi = np.array([0.0,0.0,0.0,0.0])
    r = np.array([0.0 ,0.0, 0.0, 0.0])
    dot_r = np.array([0.0, 0.0, 0.0, 0.0])
    ddot_r = np.array([0.0, 0.0, 0.0, 0.0])
    R =  np.array([1.0, 1.0, 1.0, 1.0])
    a_r = 2.0
    a_x = 2.0
    X =  np.array([0.0,0.0,0.0,0.0])

    x = np.array([0.0,0.0,0.0,0.0])
    dot_x = np.array([0.0, 0.0, 0.0, 0.0])
    ddot_x = np.array([0.0, 0.0, 0.0, 0.0])
    theta = np.array([0.0, 0.0, 0.0, 0.0])
    dt = 0.01

    n_steps = int(10 / dt)
    timestamps = np.array(range(0, n_steps+1))*dt
    log = np.zeros((n_steps, 8))

    for ind in range(n_steps):
        for i in range(4):
             dot_phi[i] = w_f[i]  + np.sum((w[i,0:3]*r[0:3]*np.sin(phi[0:3] - wphi[i,0:3])),axis=0)
             ddot_r[i] = a_r*(a_r/4.0*(R[i]-r[i]) - dot_r[i])
             ddot_x[i] = a_x*(a_x/4.0*(X[i]-x[i]) - dot_x[i])
             theta[i]  = x[i] + r[i]*np.sin(phi[i])

        # disturbance
    #     if( ind == 500)
    #         x(1) = 2;
    #         x(3) = 2;
    #         x(4) = -2;
        phi =phi+ dot_phi*dt
        dot_x =dot_x+ ddot_x * dt
        x = x+ dot_x *dt
        dot_r = dot_r + ddot_r*dt
        r = r + dot_r*dt
        #print(theta)
        log[ind,:] = np.hstack((theta,r))
    return log

        #phi = wrapTo2Pi(phi)
def ThetaToGait(theta, r):
    # use an ellipse
    theta = np.arcsin(theta)
    stride = 30
    step_height = 15
    y = stride * np.sin((theta) )
    z = step_height * np.cos( theta)
    for i in range(1,n_steps):
        for j in range(0,4):
            temp = theta[i,j] - theta[i-1,j]
            if(temp <0 ):
                z[i,j] = 0
        #temp=np.vstack([z[i,:],[0,0,0,0]])
        #z[i,:]=np.amax(temp,axis=0)
    return r*y, r*z

def InverseKinametic(x, y, z):
# 3 dof leg

    theta1 = np.arctan2(y,x)
    x_proj = x / np.cos(theta1)
    l_cf = np.sqrt(np.power(z,2) + np.power((x_proj - coxa),2))
    z = abs(z)
    ika_1 = np.arccos(z / l_cf)
    ika_2 = np.arccos(( np.power(tibia,2) - np.power(femur,2) - np.power(l_cf,2)) / (-2.0 * femur * l_cf))
    theta2 = ika_1 + ika_2
    theta3 = np.arccos(( np.power(l_cf,2) - np.power(tibia,2) - np.power(femur,2)) / (-2.0 * tibia * femur))
    m_angle = [theta1, theta2, theta3]
    return m_angle

def Transformation(theta):
    # Transformation theta represents euler angles rotation
    # x is the linear transformation of the vector v
    Rx = np.array([[1.0, 0.0, 0.0],
    [0.0, np.cos(theta[0]), -np.sin(theta[0])],
    [0.0, np.sin(theta[0]), np.cos(theta[0]) ]])

    Ry = np.array([[np.cos(theta[1]), 0.0, np.sin(theta[1])],
    [0.0, 1.0, 0.0],
    [-np.sin(theta[1]), 0.0, np.cos(theta[1])]])

    Rz = np.array([[ np.cos(theta[2]), -np.sin(theta[2]), 0.0],
    [np.sin(theta[2]), np.cos(theta[2]), 0.0],
    [0.0, 0.0, 1.0]])
    R = np.dot(Rz,np.dot(Ry,Rx))
    T = R
    return T
# Forward  k - takes  motor angles and calculates end-effector
def ForwardKinametic(theta1, theta2, theta3, body, leg_no):

    # coax start
    x1 = 0 # body(leg_no, 1)
    y1 = 0 # body(leg_no, 1)
    z1 = 0
    # Femur start

    x2 = coxa * np.cos(theta1)
    y2 = coxa * np.sin(theta1)
    z2 = 0

    # Tibia start

    d1 = femur * np.cos(theta2)

    # d1 = coxa + d1;

    x3 = x2 + d1 * np.cos(theta1)
    y3 = y2 + d1 * np.sin(theta1)
    z3 = z2 + tibia * np.sin(theta2)

    # Leg end

    z = z2 + femur * np.sin(theta2) + tibia * np.sin(theta2 + theta3)

    d1 = femur * np.cos(theta2) + tibia * np.cos(theta2 + theta3)

    # d1 = coxa + d1;

    x = x2 + d1 * np.cos(theta1)
    y = y2 + d1 * np.sin(theta1)
    temp = leg_no
    T = Transformation([0, 0, temp * pi / 2])
    X1 = np.dot(T,[x1, y1, z1])
    X2 = np.dot(T,[x2, y2, z2])
    X3 = np.dot(T,[x3, y3, z3])
    X = np.dot(T,[x, y, z])
    return X1, X2, X3, X

def FootCoordTransfrom(y, z):
    x = np.zeros(y.shape)
    for i in range(0,n_steps):
        for leg in range(0,nlegs):
            theta = leg*pi/2
            T =np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
            temp = np.dot(T, [x[i, leg], y[i, leg]])
            ## Need to fix rotation this is a hach
            if (leg  == 1 or leg == 3):
                x[i, leg] = -temp[0]
                y[i, leg] = -temp[1]
            else:
                x[i, leg] = temp[0]
                y[i, leg] = temp[1]
            #print(temp)
    return x ,y, z

#  Main
dt = 0.01
n_steps = int(10 / dt)
timestamps = np.array(range(0, n_steps+1))*dt


coxa = 37.0
femur= 70.0
tibia = 131.0

body = [[40.0, 40.0, 0.0, 0.0],
         [-40.0, 40.0, 0.0, pi/2],
         [-40.0, -40.0,0.0, pi],
        [40.0, -40.0, 0.0, 3*pi/2]]
# Oscillator
log = OscillatorFunc()
print("Oscillator Func complete...")
# Trajectory
nlegs = 4
theta=log[:,0:4]
r = log[:,4:8]
[y, z] = ThetaToGait(theta, r)

## Transform foot coord to leg frame
[x,y,z] = FootCoordTransfrom(y, z)

print(y.shape)
#x = np.zeros(y.shape)
#print(x)
print("Gait generation complete...")

# Inverse Kinematics
m_angle = np.zeros([nlegs,3,n_steps]) # sim steps, no of legs, coord
for i in range(n_steps):
    for leg_no in range(nlegs):
        [theta1, theta2, theta3] = InverseKinametic( 70.0 + x[i,leg_no], 70.0+y[i,leg_no], -100.0+z[i,leg_no])
        m_angle[leg_no, 0, i] = theta1
        m_angle[leg_no, 1, i] = theta2
        m_angle[leg_no, 2, i] = theta3


    #rad2deg(m_angle)

print("Inverse Kinemaitics calculated...")
m_coord = np.zeros([nlegs,4,3, n_steps])

for i in range(n_steps):
    for leg_no in range(nlegs):
        theta1= m_angle[leg_no,0,i]
        theta2= m_angle[leg_no, 1,i]-pi/2
        theta3= -pi+m_angle[leg_no, 2,i]
        # Should output xyz coord of each leg joint
        [X1, X2, X3, X] = ForwardKinametic(theta1,theta2,theta3, body, leg_no)
        m_coord[leg_no, 0,:,i] = X1
        m_coord[leg_no, 1,:,i] = X2
        m_coord[leg_no, 2,:,i] = X3
        m_coord[leg_no, 3,:,i] = X

    #time.sleep(0.1)

print("Forward Kinametics calculated ...")
data = np.array([m_coord[0,0:4,0,:],m_coord[0,0:4,1,:],m_coord[0,0:4,2,:]])
data1 = np.array([m_coord[1,0:4,0,:],m_coord[1,0:4,1,:],m_coord[1,0:4,2,:]])
data2 = np.array([m_coord[2,0:4,0,:],m_coord[2,0:4,1,:],m_coord[2,0:4,2,:]])
data3 = np.array([m_coord[3,0:4,0,:],m_coord[3,0:4,1,:],m_coord[3,0:4,2,:]])
#test = data
#print(test)
# Visualisation

plt.figure(1)
#plt.subplot(2,1,1)
plt.plot(log[:,0],'r',label='1',linewidth= 0.5)
plt.plot(log[:,1],'g', label='2',linewidth= 0.5)
#plt.subplot(2,1,2)
plt.plot(log[:,2],'r',label='3',linewidth= 0.5)
plt.plot(log[:,3],'g', label='4',linewidth= 0.5)
plt.title('Theta')

plt.figure(2)
#plt.subplot(2,1,1)
plt.plot(log[:,4],'r',label='1',linewidth= 0.5)
plt.plot(log[:,5],'g', label='2',linewidth= 0.5)
#plt.subplot(2,1,2)
plt.plot(log[:,6],'r',label='1',linewidth= 0.5)
plt.plot(log[:,7],'g', label='2',linewidth= 0.5)
plt.title('R')
plt.figure(3)
plt.plot(y[:,0],z[:,0],'r',label='1',linewidth= 0.5)
#plt.plot(z[:,0],'g',label='1',linewidth= 0.5)
plt.title('stride')
# Data form [[x1 x2 x3 ..], [y1 y2 y3 ..],[z1 z2 z3 ..]]...

print(data.shape)
def update_lines(num, data, lines): #improve how data is transfered

    for line in lines:
        # print(data) # remember its a line
        thisx = np.array(data[0, 0:4, num])
        thisy = np.array(data[1, 0:4, num])
        line.set_data(thisx, thisy)
        line.set_3d_properties(data[2, 0:4, num])
        line.set_marker('o')
    return lines


fig = plt.figure(4)
ax = p3.Axes3D(fig)

# NOTE: Can't pass empty arrays into 3d version of plot()
thisx = data[0, 0:3, 0]
thisy = data[1, 0:3, 0]
thisz = data[2, 0:3, 0]
lines, = [ax.plot(thisx, thisy, thisz, 'o--') ]
lines1, = [ax.plot(thisx, thisy, thisz, 'o--') ]
lines2, = [ax.plot(thisx, thisy, thisz, 'o--') ]
lines3, = [ax.plot(thisx, thisy, thisz, 'o--') ]
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.set_zlim(-100, 100)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

line_ani = animation.FuncAnimation(fig, update_lines, frames=1000, fargs=(data,lines),
                               interval=10, blit=False, repeat=True)
line_ani1 = animation.FuncAnimation(fig, update_lines, frames=1000, fargs=(data1,lines1),
                               interval=10, blit=False, repeat=True)
line_ani2 = animation.FuncAnimation(fig, update_lines, frames=1000, fargs=(data2,lines2),
                               interval=10, blit=False, repeat=True)
line_ani3 = animation.FuncAnimation(fig, update_lines, frames=1000, fargs=(data3,lines3),
                               interval=10, blit=False, repeat=True)
'''
plt.figure(5)
plt.plot(m_angle[0,0,:],'r',label='1',linewidth= 0.5)
plt.plot(m_angle[0,1,:],'g',label='2',linewidth= 0.5)
plt.plot(m_angle[0,2,:],'b',label='3',linewidth= 0.5)
plt.title('Test angle')

plt.figure(6)
plt.plot(y[:,0],'r',label='1',linewidth= 0.5)
plt.plot(z[:,0],'g',label='2',linewidth= 0.5)
#plt.plot(m_angle[0,2,:],'b',label='3',linewidth= 0.5)
plt.title('Test Leg Coord')
'''
plt.show()