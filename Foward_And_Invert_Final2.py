import numpy as np
import math as mt 


##DH PARAMETER##
#[theta d a alpha]
#0 L1 0 0
#0 0 0 alpha1
#0 0 L2 0
#0 0 L3 0
#0 L5 0 -alpha2

##------------------------Foward Kinematics----------------------------------------##
L1 = 10; L2=10; L3=10; L4 = 10; L5 = 20 # Link length
alpha1 = 0; alpha2=mt.pi/2; alpha3 = 0; alpha4=0; alpha5 = (-1)*mt.pi/2 #Twist angle
th1 = 27; th2=36; th3=25; th4= 15; th5= 2 # Joint angle

def foward_kinematics (th1,th2,th3,th4,th5):
    T01 = np.array([[mt.cos(mt.radians(th1)), (-1) * mt.sin(mt.radians(th1)), 0 ,0 ],
    [mt.sin(mt.radians(th1)), mt.cos(mt.radians(th1)), 0 ,0],
    [0, 0 , 1 , L1], 
    [0, 0 , 0 , 1]] )

    T12 = np.array([[mt.cos(mt.radians(th2)), (-1) * mt.sin(mt.radians(th2)), 0 ,0 ],
    [0 ,0 ,-1 ,0],
    [mt.sin(mt.radians(th2)), mt.cos(mt.radians(th2)) , 0 , 0], 
    [0, 0 , 0 , 1]] )

    T23 = np.array([[mt.cos(mt.radians(th3)), (-1) * mt.sin(mt.radians(th3)), 0 ,L2 ],
    [mt.sin(mt.radians(th3)), mt.cos(mt.radians(th3)), 0 ,0],
    [0, 0 , 1 , 0], 
    [0, 0 , 0 , 1]] )

    T34 = np.array([[mt.cos(mt.radians(th4)), (-1) * mt.sin(mt.radians(th4)), 0 ,L3 ],
    [mt.sin(mt.radians(th4)), mt.cos(mt.radians(th4)), 0 ,0],
    [0, 0 , 1 , 0], 
    [0, 0 , 0 , 1]] )

    T45 = np.array([[mt.cos(mt.radians(th5)), (-1) * mt.sin(mt.radians(th5)), 0 ,0 ],
    [0 ,0 ,1 ,L5],
    [(-1) *mt.sin(mt.radians(th5)), (-1) *mt.cos(mt.radians(th5)) , 0 , 0], 
    [0, 0 , 0 , 1]] )

    t02 = np.dot(T01,T12); t03 = np.dot(t02,T23); t04 = np.dot(t03,T34); t05 = np.dot(t04,T45)
    
    return t05


Tmatrix_05 = foward_kinematics(th1,th2,th3,th4,th5) 
print(Tmatrix_05)




##------------------------Inverse Kinematics----------------------------------------##
r11 = Tmatrix_05[0,0]; r12 = Tmatrix_05[0,1]; r13 = Tmatrix_05[0,2]; px = Tmatrix_05[0,3] 
r21 = Tmatrix_05[1,0]; r22 = Tmatrix_05[1,1]; r23 = Tmatrix_05[1,2]; py = Tmatrix_05[1,3]
r31 = Tmatrix_05[2,0]; r32 = Tmatrix_05[2,1]; r33 = Tmatrix_05[2,2]; pz = Tmatrix_05[2,3]


def inverse_theta1 (p_x, p_y, th2, th_3):
    w_x = p_x - (L5*r13)
    w_y = p_y - (L5*r23)
    if ( th2 == 90 ) and ( th3 == 0 ):
        theta1 = mt.atan (p_y/p_x)
    elif (th2 == 90 ) and (th3 == 180 ):
        theta1 = mt.atan (p_y/p_x)
    else:
        if w_x == 0:
            theta1 = mt.pi/2
        else:
            theta1 = mt.atan2(w_y,w_x)
    theta1 = mt.degrees(theta1)
    if theta1 >= 0:
        theta1 = theta1
    else:
        theta1 += 180
    theta1r = mt.radians (theta1)
    return theta1, theta1r

def inverse_theta2 (p_x, p_y, p_z, th_3):
    w_x = p_x - (L5*r13)
    w_y = p_y - (L5*r23)
    w_z = p_z - (L5*r33)
    s = w_z - L1
    r = mt.sqrt((w_x**2)+(w_y**2)) 
    D = mt.sqrt(r**2 + s**2)
    cos_beta = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
    cos_beta = max(-1.0, min(1.0, cos_beta)) 
    beta = mt.acos(cos_beta) 
    alpha = mt.atan2(s, r) # alpha ในหน่วยเรเดียน (s คือ Z, r คือ XY projection)

    theta2r = alpha - beta 
    
    theta2 = mt.degrees(theta2r)
    
    return theta2, theta2r
    
def inverse_theta3 (p_x, p_y, p_z):
    w_x = p_x - (L5*r13)
    w_y = p_y - (L5*r23)
    w_z = p_z - (L5*r33)
    s = w_z - L1
    r = mt.sqrt((w_x**2)+(w_y**2))
    c3 = round (((s**2)+(r**2)-(L2**2)-(L3**2)) / (2*L2*L3),6)
    theta3 = mt.acos (c3)
    theta3 = mt.degrees(theta3)
    if theta3 >= 0:
        theta3 = theta3
    else:
        theta3 += 180
    theta3r = mt.radians(theta3)
    return theta3, theta3r

def inverse_theta45(t05, th1 ,th2, th3):
    r_05 = t05[:-1, :-1]
    t01 = np.array([[mt.cos(mt.radians(th1)), (-1) * mt.sin(mt.radians(th1)), 0 ,0 ],
    [mt.sin(mt.radians(th1)), mt.cos(mt.radians(th1)), 0 ,0],
    [0, 0 , 1 , L1], 
    [0, 0 , 0 , 1]] )

    t12 = np.array([[mt.cos(mt.radians(th2)), (-1) * mt.sin(mt.radians(th2)), 0 ,0 ],
    [0 ,0 ,-1 ,0],
    [mt.sin(mt.radians(th2)), mt.cos(mt.radians(th2)) , 0 , 0], 
    [0, 0 , 0 , 1]] )

    t23 = np.array([[mt.cos(mt.radians(th3)), (-1) * mt.sin(mt.radians(th3)), 0 ,L2 ],
    [mt.sin(mt.radians(th3)), mt.cos(mt.radians(th3)), 0 ,0],
    [0, 0 , 1 , 0], 
    [0, 0 , 0 , 1]] )

    t02 = np.dot(t01,t12); t03 = np.dot(t02,t23)

    r_03 = t03[:-1, :-1]
    inv_r03 = np.transpose(r_03)
    r_35 = np.dot(inv_r03,r_05)

    sin_4 = r_35[0,2]
    cos4 = r_35[1,2]
    sin4 = (-1)*sin_4
    theta4r = mt.atan2 (sin4,cos4)
    sin5 = (-1)*r_35[2,0]
    cos5 = (-1)*r_35[2,1]
    theta5r = mt.atan2 (sin5,cos5)
    theta4 = mt.degrees (theta4r)
    theta5 = mt.degrees (theta5r)
    return theta4, theta5 




th_3, th_3r = inverse_theta3 (px,py,pz)
theta_3 = th_3; theta_3r = th_3r

th_2, th_2r = inverse_theta2 (px,py,pz,theta_3r)
theta_2 = th_2; theta_2r = th_2r

th_1, th_1r = inverse_theta1 (px,py,theta_2,theta_3)
theta_1 = th_1; theta_1r = th_1r

th_4, th_5 = inverse_theta45 (Tmatrix_05, theta_1 ,theta_2, theta_3)
theta_4 = th_4
theta_5 = th_5


print(round(theta_1))
print(round(theta_2))
print(round(theta_3))
print(round(theta_4))
print(round(theta_5))


