## Projet BLOC 3
import matplotlib.pyplot as plt
import numpy as np
import math


## Definir les amers
def amers(Nb_amer : int, x0 : float, y0 : float) :
    x = x0
    y = y0
    m = np.ndarray((Nb_amer, 2))
    
    xa = np.ndarray(Nb_amer)
    ya = np.ndarray(Nb_amer)
    if (Nb_amer % 2 == 0) :
        for i in range(int(Nb_amer / 2 )) :
            m[i] = [x, y]
            x = x + 2
        i = i + 1
        x = x - 2
        y = y + 2
        while i < Nb_amer :
            m[i] = [x, y]
            x = x - 2
            i = i + 1
    else :
        for i in range(int(Nb_amer / 2)) :
            m[i] = [x, y]
            x = x + 2
        i = i + 1
        y = y + 1
        m[i] = [x, y]
        
        i = i + 1
        x = x - 2
        y = y + 1
        while (i < Nb_amer) :
            m[i] = [x, y]
            x = x - 2
            i = i + 1
        
    xa = m[:,0]
    ya = m[:,1]

    pos_a = np.concatenate(m)

    incertitude_amer = np.diag(0.00001*np.ones(2*Nb_amer))
    pos_a = pos_a + (np.linalg.cholesky(incertitude_amer))@(np.random.normal(size=(16)))
    
    return (pos_a,Nb_amer)


#definir le robot
def deplacement_robot(r : float, u : float, v : float):
    if(u[0,1] == 0):
        r[0,0] = r[0,0] + (u[0,0]) * math.cos(r[0,2])
        r[0,1] = r[0,1] + (u[0,0]) * math.sin(r[0,2])
        r[0,2] = r[0,2] + u[0,1]
    else:
        r[0,0] = r[0,0] + (u[0,0]/u[0,1]) * (math.sin(r[0,2]+u[0,1]) - math.sin(r[0,2]))
        r[0,1] = r[0,1] + (u[0,0]/u[0,1]) * (math.cos(r[0,2]) - math.cos(r[0,2]+u[0,1]))
        r[0,2] = r[0,2] + u[0,1]
    r[0,0] = r[0,0] + v[0]
    r[0,1] = r[0,1] + v[1]
    r[0,2] = r[0,2] + v[2]
    return (r)

#affichage
def affichage(pos_a:float, r:float, T:int):
    # Affichage de la map
    i = 0
    while(i < int(pos_a.shape[0])):
        plt.scatter(pos_a[i], pos_a[i+1])
        i = i + 2
    
    # Affichage de la trajectoire du robot dans la map
    plt.plot(r[:, 0], r[:, 1])
    # Affichage du robot dans son dernier etat
    plt.scatter(r[T, 0], r[T, 1])                     #remplacer t par le dernier indice du vecteur Temps

    #print('u.shape : ', u.shape)
    #print('r.shape : ', r.shape)
    
    plt.show()

#Generation des observations
def generation_mesure(z1:float, r1:float, pos_a:float, v:float):
    for e in range(Nb_amer):
        z1[0, e*2] = math.sqrt((r1[0,0]-pos_a[2*e])**2 + (r1[0,1]-pos_a[2*e+1])**2) + v[e]
        z1[0, 2*e+1] = math.atan2(pos_a[2*e+1]-r1[0,1], pos_a[2*e]-r1[0,0]) - r1[0,2] + v[e+1]

        if (z1[0,e*2] > 4 or abs(z1[0,e*2+1])>np.pi/4):
            z1[0, e * 2] = np.nan
            z1[0, e * 2 + 1] = np.nan

    #print("z1.shape : ", z1.shape)
    return z1


## Initialisation
x_r = 0.1
y_r = 0
theta_r = 0
t = 0
r = np.array([[x_r, y_r, theta_r]])
r1 = np.array([[x_r, y_r, theta_r]])
u = np.array([[1, 0]])
(pos_a, Nb_amer) = amers(8, 1, -0.5)    # 8 amers, position du premier a (1,1)
i = 0
T = 0

H = np.array([[ 1 , 0 , 0, 0, 0 ] , [ 0 , 1 , 0, 0, 0 ] , [ 0 , 0 , 1, 0, 0 ]])

Qw = np.diag([0.000001, 0.000001, 0.000001])
w = np.transpose((np.linalg.cholesky(Qw))@(np.random.normal(size=(3,39))))
Rv = np.diag(0.000001*np.ones(2*Nb_amer))
v = np.transpose((np.linalg.cholesky(Rv))@(np.random.normal(size=(2*Nb_amer,39))))


z = np.array([np.ones(2*Nb_amer)])
z = generation_mesure(z, r1, pos_a, v[i])
z1 = np.array([np.ones(2*Nb_amer)])


#Nb_amer pair
if Nb_amer % 2 == 0:
    
    dist = r[0,0] - pos_a[Nb_amer-2]     # x_robot - x_amer_(4)
    
    #Aller en x
    while(dist<0):
        u1 = np.array([[1, 0]])
        r1 = deplacement_robot(r1,u1,w[i])
        r = np.concatenate((r, r1), axis = 0)
        u = np.concatenate((u, u1), axis=0)
        z1 = generation_mesure(z1, r1, pos_a, v[i])
        z = np.concatenate((z, z1), axis=0)
        
        dist = r[i,0] - pos_a[Nb_amer-2]
        i += 1

    #Rotation
    while(r[i,2] < np.pi*0.99):
        u1 = np.array([[0.5 , np.pi/8]])
        r1 = deplacement_robot(r1,u1,w[i])
        r = np.concatenate((r, r1), axis = 0)
        u = np.concatenate((u, u1), axis=0)
        z1 = generation_mesure(z1, r1, pos_a, v[i])
        z = np.concatenate((z, z1), axis=0)

        dist_y = r[i,1] - pos_a[Nb_amer+1]
        i += 1
    
    #Retour en x
    dist = r[i,0] - pos_a[0]
    while(dist>0):
        u1 = np.array([[1 , 0]])
        r1 = deplacement_robot(r1,u1,w[i])
        r = np.concatenate((r, r1), axis = 0)
        u = np.concatenate((u, u1), axis = 0)
        z1 = generation_mesure(z1, r1, pos_a, v[i])
        z = np.concatenate((z, z1), axis=0)

        dist = r[i,0] - pos_a[0]
        i += 1

    # Rotation
    while(r[i,2] < np.pi*2*0.99):
        u1 = np.array([[0.5, np.pi / 8]])
        r1 = deplacement_robot(r1, u1, w[i])
        r = np.concatenate((r, r1), axis=0)
        u = np.concatenate((u, u1), axis=0)
        z1 = generation_mesure(z1, r1, pos_a, v[i])
        z = np.concatenate((z, z1), axis=0)

        i += 1

    #Aller 2 en x 
    dist = r[i, 0] - pos_a[2]
    while(dist<0):
        u1 = np.array([[1, 0]])
        r1 = deplacement_robot(r1, u1, w[i])
        r = np.concatenate((r, r1), axis=0)
        u = np.concatenate((u, u1), axis=0)
        z1 = generation_mesure(z1, r1, pos_a, v[i])
        z = np.concatenate((z, z1), axis=0)
        
        dist = r[i, 0] - pos_a[2]
        i += 1
    #print("r apres x aller 2 : ", r)
    print("r.shape : ", r.shape)
    
    T = i
"""
#Nb_amer impair
else:

    dist = r[i,0] - xa[int(Nb_amer/2)]

    #Aller en x
    while(dist<0):
        u1 = np.array([[1 , 0 , 0]])

        r1 = deplacement_robot(r1,u1)
        r = np.concatenate((r, r1), axis = 0)
        u = np.concatenate((u, u1), axis=0)

        dist = r[i,0] - xa[int(Nb_amer/2)]
        i += 1

    #Rotation
    u1 = np.array([[0 , 0 , np.pi/2]])

    r1 = deplacement_robot(r1,u1)
    r = np.concatenate((r, r1), axis = 0)
    u = np.concatenate((u, u1), axis=0)

    i += 1

    #Aller en y
    dist = r[i,1] - ya[int(Nb_amer/2)+1]
    while(dist<0):
        u1 = np.array([[0 , 1 , 0]])

        r1 = deplacement_robot(r1,u1)
        r = np.concatenate((r, r1), axis = 0)
        u = np.concatenate((u, u1), axis=0)

        dist = r[i,1] - ya[int(Nb_amer/2)+1]
        i += 1

    #Rotation
    u1 = np.array([[0 , 0 , np.pi/2]])

    r1 = deplacement_robot(r1,u1)
    r = np.concatenate((r, r1), axis = 0)
    u = np.concatenate((u, u1), axis=0)

    i += 1

    #Retour en x
    dist = r[i,0] - xa[0]
    while(dist>0):
        u1 = np.array([[-1 , 0 , 0]])

        r1 = deplacement_robot(r1,u1)
        r = np.concatenate((r, r1), axis = 0)
        u = np.concatenate((u, u1), axis=0)
    
        dist = r[i,0] - xa[0]
        i += 1

    # Rotation
    u1 = np.array([[0, 0, np.pi / 2]])

    r1 = deplacement_robot(r1, u1)
    r = np.concatenate((r, r1), axis=0)
    u = np.concatenate((u, u1), axis=0)

    i += 1

    # Retour en y
    dist = r[i, 1] - ya[Nb_amer-1]

    while (dist > 0):
        u1 = np.array([[0, -1, 0]])

        r1 = deplacement_robot(r1, u1)
        r = np.concatenate((r, r1), axis=0)
        u = np.concatenate((u, u1), axis=0)

        dist = r[i, 1] - ya[Nb_amer-1]

        i += 1
    T = i
"""

#print("T : ", T)


#Création du vecteur d'etat
X = np.array([np.concatenate((r[0], pos_a), axis=0)])
i = 1
while(i<=T):
    inter = np.array([np.concatenate((r[i], pos_a), axis=0)])
    X = np.concatenate((X, inter), axis=0)
    i += 1
print("X.shape : ", X.shape)

print("z.shape : ", z.shape)
#print("z[0] : ", z[0])
#print("z[1] : ", z[1])



affichage(pos_a, r, T)

print("w.shape : ", w.shape)
print("v.shape : ", v.shape)