## Projet BLOC 3
import matplotlib.pyplot as plt
import numpy as np
import time



# Plutot definir une loi avec xk+1, fk+1, uk pour le robot


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
        y = y - 2
        while i < Nb_amer :
            m[i] = [x, y]
            x = x - 2
            i = i + 1
    else :
        for i in range(int(Nb_amer / 2)) :
            m[i] = [x, y]
            x = x + 2
        i = i + 1
        y = y - 1
        m[i] = [x, y]
        
        i = i + 1
        x = x - 2
        y = y - 1
        while (i < Nb_amer) :
            m[i] = [x, y]
            x = x - 2
            i = i + 1
        
    xa = m[:,0]
    ya = m[:,1]
    
    return (xa,ya,Nb_amer)


#definir le robot
def robot(r : float, u : float):
    r = r + u
    return (r)


## Initialisation
x_r = 0
y_r = 0
theta_r = 0
t=0
r = np.array([[x_r, y_r, theta_r]])
r1 = r
(xa,ya,Nb_amer)=amers(8,0,1)    # 8 amers, position du premier a (0,1)



#main

dist = 0
i = 0
if Nb_amer % 2 == 0 :

    #Aller en x

    while(dist<1) :
        u = np.array([[1 , 0 , 0]])

        r1 = robot(r1,u)
        r = np.concatenate((r, r1), axis = 0)
        
        dist = r[i,0] - xa[int(Nb_amer/2-1)]
        i += 1

    #Rotation

    u = np.array([[0 , 0 , np.pi/2]])

    r1 = robot(r1,u)
    r = np.concatenate((r, r1), axis = 0)

    i += 1

    #Aller en y

    dist = 0
    while(dist<=1):
        u = np.array([[0 , 1 , 0]])

        r1 = robot(r1,u)
        r = np.concatenate((r, r1), axis = 0)

        dist = r[i,1] - ya[int(Nb_amer/2)]

        i += 1


    #Rotation

    u = np.array([[0 , 0 , np.pi/2]])

    r1 = robot(r1,u)
    r = np.concatenate((r, r1), axis = 0)

    i += 1

    #Retour en x

    dist = r[i,0] 
    while(dist>1):
        u = np.array([[-1 , 0 , 0]])

        r1 = robot(r1,u)
        r = np.concatenate((r, r1), axis = 0)

        dist = r[i,0] - xa[0]
        i += 1    
else :
    while(dist<1) :
        u = np.array([[1 , 0 , 0]])

        r1 = robot(r1,u)
        r = np.concatenate((r, r1), axis = 0)

        dist = r[i,0] - xa[int(Nb_amer/2)]
        i += 1
    u = np.array([[0 , 0 , np.pi/2]])

    r1 = robot(r1,u)
    r = np.concatenate((r, r1), axis = 0)

    i += 1
    dist = 0
    while(dist<=0):
        u = np.array([[0 , 1 , 0]])

        r1 = robot(r1,u)
        r = np.concatenate((r, r1), axis = 0)

        dist = r[i,1] - ya[int(Nb_amer/2)+1]
        i += 1
    u = np.array([[0 , 0 , np.pi/2]])

    r1 = robot(r1,u)
    r = np.concatenate((r, r1), axis = 0)

    i += 1
    dist = r1[i,0] - xa[0]
    while(dist>0):
        u = np.array([[-1 , 0 , 0]])

        r1 = robot(r1,u)
        r = np.concatenate((r, r1), axis = 0)
    
        dist = r[i,0] - xa[0]
        i += 1

# Affichage
# Affichage de la map
plt.scatter(xa, ya)
# Affichage de la trajectoire du robot dans la map
plt.plot(r[:, 0], r[:, 1])

plt.show()
print(u.shape)