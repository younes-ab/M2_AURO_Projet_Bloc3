## Projet BLOC 3
import matplotlib.pyplot as plt
import numpy as np
import math


## Definir les amers
def amers(Nb_amer : int, x0 : float, y0 : float, incertitude_amer : float) :
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

    pos_a = pos_a + (np.linalg.cholesky(incertitude_amer))@(np.random.normal(size=(16)))
    
    return pos_a


#definir le robot
def deplacement_robot(r : float, u : float, w : float):
    if(u[0,1] == 0):
        r[0,0] = r[0,0] + (u[0,0]) * math.cos(r[0,2])
        r[0,1] = r[0,1] + (u[0,0]) * math.sin(r[0,2])
        r[0,2] = r[0,2] + u[0,1]
    else:   
        r[0,0] = r[0,0] + (u[0,0]/u[0,1]) * (math.sin(r[0,2]+u[0,1]) - math.sin(r[0,2])) 
        r[0,1] = r[0,1] + (u[0,0]/u[0,1]) * (math.cos(r[0,2]) - math.cos(r[0,2]+u[0,1]))
        r[0,2] = r[0,2] + u[0,1]
    r[0,0] = r[0,0] + w[0]
    r[0,1] = r[0,1] + w[1]
    r[0,2] = r[0,2] + w[2]
    return (r)

#affichage
def affichage(pos_a:float, r:float, T:int, r_maj_tab:float):
    # Affichage de la map
    i = 0
    while(i < int(pos_a.shape[0])):
        plt.scatter(pos_a[i], pos_a[i+1])
        i = i + 2
    
    # Affichage de la trajectoire du robot dans la map
    plt.plot(r[:, 0], r[:, 1])
    # Affichage du robot dans son dernier etat
    plt.scatter(r[T, 0], r[T, 1])                     #remplacer t par le dernier indice du vecteur Temps
    # Affichage des états robot
    plt.plot(r_maj_tab[:, 0], r_maj_tab[:, 1])
    # Affichage du robot dans son dernier etat predit
    plt.scatter(r_maj_tab[T, 0], r_maj_tab[T, 1])
    
    plt.show()

#Generation des observations
def generation_mesure(z1:float, r1:float, pos_a:float, v:float):
    for e in range(Nb_amer):
        z1[0, e*2] = math.sqrt((pos_a[2*e]-r1[0,0])**2 + (pos_a[2*e+1]-r1[0,1])**2) + v[e]
        z1[0, 2*e+1] = (math.atan2(pos_a[2*e+1]-r1[0,1], pos_a[2*e]-r1[0,0]) + v[e+1] - r1[0,2]) %(np.pi*2)

        if (z1[0,e*2] > 4 or abs(z1[0,e*2+1])>np.pi/2):
            z1[0, e * 2] = np.nan
            z1[0, e * 2 + 1] = np.nan
    print("z1 : ", z1)
    print("z1.shape : ", z1.shape)
    return z1


#Creation de la matrice F
def F_san(r_pred : float , u : float):
    F_sa = np.zeros((19,19))
    if(u[0,1] == 0):
        F_sa[0,0] = 1
        F_sa[0,2] = -u[0,0] * math.sin(r_pred[0,2])
        F_sa[1,1] = 1
        F_sa[1,2] = u[0,0] * math.cos(r_pred[0,2])
        F_sa[2,2] = 1
    else:
        F_sa[0,0] = 1
        F_sa[0,2] = (u[0,0]/u[0,1]) * ( math.cos(r_pred[0,2]+u[0,1]) - math.cos(r_pred[0,2]) )
        F_sa[1,1] = 1
        F_sa[1,2] = (u[0,0]/u[0,1]) * ( math.sin(r_pred[0,2]+u[0,1]) - math.sin(r_pred[0,2]) )
        F_sa[2,2] = 1
    return F_sa


## Initialisation
x_r = 0.1
y_r = 0
theta_r = 0
t = 0
r = np.array([[x_r, y_r, theta_r]])
r1 = np.array([[x_r, y_r, theta_r]])
u = np.array([[1, 0]])
Nb_amer = 8
i = 0

Qw = np.diag([0.00001, 0.00001, 0.00001])
w = np.transpose((np.linalg.cholesky(Qw))@(np.random.normal(size=(3,39))))
Rv = np.diag(0.0001*np.ones(2*Nb_amer))
v = np.transpose((np.linalg.cholesky(Rv))@(np.random.normal(size=(2*Nb_amer,39))))              
incertitude_amer = np.diag(0.000001*np.ones(2*Nb_amer))

pos_a = amers(Nb_amer, 1, -0.5, incertitude_amer)    # 8 amers, position du premier a (1,1)

Qw_plus = np.zeros((3,16))
incertitude_amer_plus = np.zeros((16,3))
Qw_plus = np.concatenate((Qw,Qw_plus), axis = 1)
incertitude_amer_plus = np.concatenate((incertitude_amer_plus,incertitude_amer), axis = 1)
Qw_real = np.concatenate((Qw_plus,incertitude_amer_plus), axis = 0)

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
    while(r[i,2] < np.pi*0.95):
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
    while(r[i,2] < np.pi*2*0.95):
        u1 = np.array([[0.5, np.pi / 8]])
        r1 = deplacement_robot(r1, u1, w[i])
        r = np.concatenate((r, r1), axis=0)
        u = np.concatenate((u, u1), axis=0)
        z1 = generation_mesure(z1, r1, pos_a, v[i])
        z = np.concatenate((z, z1), axis=0)

        i += 1

    #Aller 2 en x 
    dist = r[i, 0] - pos_a[2]
    while(dist<0 or i<38):
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
    
    T = i           #Temps
    N = i + 1       #N iterations

#Création du vecteur d'etat
X = np.array([np.concatenate((r[0], pos_a), axis=0)])
i = 1
while(i<=T):
    inter = np.array([np.concatenate((r[i], pos_a), axis=0)])
    X = np.concatenate((X, inter), axis=0)
    i += 1

print("X.shape : ", X.shape)
print("z.shape : ", z.shape)
print("z : ", z)
print("w.shape : ", w.shape)
print("v.shape : ", v.shape)


#Initialisation

x_pred = np.zeros((1,19))
P_pred = np.zeros((19,19))
x_maj = np.zeros((1,19))
P_maj = np.zeros((19,19))
K = np.zeros((19,16))
r_pred = np.zeros((1,3))
r_maj = np.zeros((1,3))
F = np.identity(19)

r_maj_tab = np.zeros((1,3))
P_maj_tab = np.zeros((19,19))


print("r_pred : ", r_pred.shape)
print("pos_a : ", pos_a.shape)
print("w.shape : ", w.shape)
print("u.shape : ", u.shape)



for i in range (N):
    print("\nInstant n°",i)

    # Recup obs sans nan ex:z_use
    amers_visibles = []
    for j in range(Nb_amer):
        if(np.isnan(z[i][2*j]) == False):
            amers_visibles = amers_visibles + [j]
    print("amers_visibles : ", amers_visibles)

    #r_pred = deplacement_robot(r_maj, u[[0]], w[i]) - w[[i]]
    if (u[0, 1] == 0):
        r_pred[0, 0] = r_maj[0, 0] + (u[0, 0]) * math.cos(r_maj[0, 2])
        r_pred[0, 1] = r_maj[0, 1] + (u[0, 0]) * math.sin(r_maj[0, 2])
        r_pred[0, 2] = r_maj[0, 2] + u[0, 1]
    else:
        r_pred[0, 0] = r_maj[0, 0] + (u[0, 0] / u[0, 1]) * (math.sin(r_maj[0, 2] + u[0, 1]) - math.sin(r_maj[0, 2]))
        r_pred[0, 1] = r_maj[0, 1] + (u[0, 0] / u[0, 1]) * (math.cos(r_maj[0, 2]) - math.cos(r_maj[0, 2] + u[0, 1]))
        r_pred[0, 2] = r_maj[0, 2] + u[0, 1]

    x_pred = np.concatenate((r_pred, [pos_a]), axis=1)
    #print ("x_pred : ", x_pred)

    F = F_san(r_pred, u)
    #print ("F : ", F)

    P_pred = F @ P_maj @ F.T + Qw_real
    #print ("P_pred[0] : ", P_pred[0])
    #print ("P_maj[0] : ", P_maj[0])


    z_visible = np.zeros((1, len(amers_visibles * 2)))
    z_pred = np.zeros((1, len(amers_visibles * 2)))
    H = np.zeros((len(amers_visibles*2), 19))
    Rv = np.zeros((len(amers_visibles * 2), len(amers_visibles * 2)))
    for o in range(len(amers_visibles)):
        Rv[2 * o][2 * o] = 0.01
        Rv[2 * o + 1][2 * o + 1] = np.pi/10
    Rv = np.diag(0.0001 * np.ones(len(amers_visibles*2)))

    for e in range(len(amers_visibles)):
        z_visible[0, e * 2] = z[i][2 * amers_visibles[e]]
        z_visible[0, 2 * e + 1] = z[i][2 * amers_visibles[e] + 1]
        z_pred[0, e * 2] = math.sqrt((r_pred[0, 0] - pos_a[amers_visibles[e]-1]) ** 2 + (r_pred[0, 1] - pos_a[amers_visibles[e]]) ** 2)
        z_pred[0, 2 * e + 1] = math.atan2(pos_a[amers_visibles[e]] - r_pred[0, 1], pos_a[amers_visibles[e]-1] - r_pred[0, 0]) - r_pred[0, 2]

        # H[2*e, 0] = (2 * r_pred[0, 0] - 2 * pos_a[2 * amers_visibles[e]]) / (2 * math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        # H[2*e, 1] = (2 * r_pred[0, 1] - 2 * pos_a[2 * amers_visibles[e] + 1]) / (2 * math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        # H[2*e, 3+2*amers_visibles[e]] = (-2 * r_pred[0, 0] + 2 * pos_a[2 * amers_visibles[e]]) / (2 * math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        # H[2*e, 4+2*amers_visibles[e]] = (-2 * r_pred[0, 1] + 2 * pos_a[2 * amers_visibles[e] + 1]) / (2 * math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        #
        # H[2*e+1, 0] = ((pos_a[2 * amers_visibles[e] + 1] - r_pred[0, 1]) / (pos_a[2 * amers_visibles[e]] - r_pred[0, 0]) ** 2) / (1 + (math.atan2(pos_a[2 * amers_visibles[e] + 1] - r_pred[0, 1], pos_a[2 * amers_visibles[e]] - r_pred[0, 0])) ** 2)
        # H[2*e+1, 1] = -(1 / (pos_a[2 * amers_visibles[e]] - r_pred[0, 0])) / (1 + (math.atan2(pos_a[2 * amers_visibles[e] + 1] - r_pred[0, 1], pos_a[2 * amers_visibles[e]] - r_pred[0, 0])) ** 2)
        # H[2*e+1, 2] = -1
        # H[2*e+1, 3+2*amers_visibles[e]] = (-(pos_a[2 * amers_visibles[e] + 1] - r_pred[0, 1]) / (pos_a[2 * amers_visibles[e]] - r_pred[0, 0]) ** 2) / (1 + (math.atan2(pos_a[2 * amers_visibles[e] + 1] - r_pred[0, 1], pos_a[2 * amers_visibles[e]] - r_pred[0, 0])) ** 2)
        # H[2*e+1, 4+2*amers_visibles[e]] = (1 / (pos_a[2 * amers_visibles[e]] - r_pred[0, 0])) / (1 + (math.atan2(pos_a[2 * amers_visibles[e] + 1] - r_pred[0, 1], pos_a[2 * amers_visibles[e]] - r_pred[0, 0])) ** 2)


        H[2*e, 0] = -2 * (pos_a[amers_visibles[e]] - r_pred[0,0]) * 1 / (2*math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        H[2 * e, 1] = -2 * (pos_a[amers_visibles[e] + 1] - r_pred[0, 1]) / (2 * math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        H[2 * e, 3 + 2 * amers_visibles[e]] = 2 * (pos_a[amers_visibles[e]] - r_pred[0, 0]) / (2 * math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        H[2 * e, 4 + 2 * amers_visibles[e]] = 2 * (pos_a[amers_visibles[e] + 1] - r_pred[0, 1]) / (2 * math.sqrt((r_pred[0, 0] - pos_a[2 * amers_visibles[e]]) ** 2 + (r_pred[0, 1] - pos_a[2 * amers_visibles[e] + 1]) ** 2))
        #
        H[2 * e + 1, 0] = (pos_a[amers_visibles[e] + 1] - r_pred[0,2]) / ((pos_a[amers_visibles[e]] - r_pred[0,1]) **2 + (pos_a[amers_visibles[e] + 1] - r_pred[0,2]) **2)
        H[2 * e + 1, 1] = -(pos_a[amers_visibles[e]] - r_pred[0,1]) / ((pos_a[amers_visibles[e]] - r_pred[0,1]) **2 + (pos_a[amers_visibles[e] +1] - r_pred[0,2]) **2)
        H[2 * e + 1, 2] = -1
        H[2 * e + 1, 3 + 2 * amers_visibles[e]] = -(pos_a[amers_visibles[e] +1] - r_pred[0,2]) / ((pos_a[amers_visibles[e]] - r_pred[0,1]) **2 + (pos_a[amers_visibles[e] +1] - r_pred[0,2])**2)
        H[2*e+1, 4+2*amers_visibles[e]] = (pos_a[amers_visibles[e]] - r_pred[0,1]) / ((pos_a[amers_visibles[e]] - r_pred[0,1])**2 + (pos_a[amers_visibles[e] +1] - r_pred[0,2])**2)
    # print ("H[0] : ", H[0])
    print("z_pred : ", z_pred)
    print("z_visible : ", z_visible)
    print("r_pred : ", r_pred)

    S = Rv + H @ P_pred @ np.transpose(H)
    # print ("S : ", S)

    K = P_pred @ H.T @ np.linalg.inv(S)
    #print("K.shape : ", K.shape)

    #x_maj = x_pred + K @ (z[i]-z_pred[0])
    #print ("K.shape : ", K.shape)
    #print ("z_visible[i]-z_pred[0] : ", z_visible[i]-z_pred[0])
    #print("z[i].shape : ", z[i].shape)
    x_maj = x_pred + K @ (z_visible[0] - z_pred[0])
    print ("x_maj : ", x_maj)
    #print ("K @ (z[i]-z_pred[0]) : ", K @ (z[i]-z_pred[0]))
    

    r_maj[0] = x_maj[0,0:3]
    P_maj = P_pred - K @ H @ P_pred
    if(i==0):
        r_maj_tab = r_maj
        P_maj_tab = P_maj
    else:
        r_maj_tab = np.concatenate((r_maj_tab,r_maj), axis = 0)
        P_maj_tab = np.concatenate((P_maj_tab, P_maj), axis=0)


#print("K.shape : ", K.shape)
print("z.shape : ", z.shape)
print("z[i] : ", z[i])
print("u : ", u.shape)
#print("x_maj : ", x_maj)
print ("r_maj_tab.shape : ", r_maj_tab.shape)
#print ("r_maj_tab : ", r_maj_tab)

print("T : ", T)
print("N iterations : ", N)

affichage(pos_a, r, T, r_maj_tab)

